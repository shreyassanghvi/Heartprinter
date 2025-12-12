# Heartprinter State Machine Documentation

This document describes the state machine that controls the Heartprinter C++ system.

---

## Table of Contents

1. [Overview](#overview)
2. [State Definitions](#state-definitions)
3. [State Transitions](#state-transitions)
4. [State Diagrams](#state-diagrams)
5. [Implementation Details](#implementation-details)

---

## Overview

The Heartprinter system uses a **finite state machine** (FSM) to manage system behavior and ensure safe, predictable operation. The state machine runs in the main control loop and processes one state transition per iteration.

### Key Characteristics

- **Deterministic**: Each state has well-defined entry, execution, and exit logic
- **Safe**: Safety checks prevent dangerous state transitions
- **Recoverable**: Error state allows controlled recovery
- **Logged**: All state transitions are logged for debugging

### State Enum

```cpp
enum class States {
    START,          // 0: Initial state
    INITIALIZING,   // 1: Hardware initialization
    CALIBRATION,    // 2: Automatic base detection
    READY,          // 3: Idle, waiting for commands
    RUNNING,        // 4: Position achieved, monitoring
    MOVING,         // 5: Actively moving to target
    TENSION,        // 6: Adjusting cable tension
    ERR,            // 7: Error state
    END,            // 8: Shutdown requested
    CLEANUP         // 9: Resource cleanup
};
```

---

## State Definitions

### 1. START

**Purpose**: System entry point

**Entry Conditions**:
- System just started
- Constructor completed

**Actions**:
- Log system start
- Initialize logging system
- Set initial variables

**Exit Conditions**:
- Always transitions to INITIALIZING

**Next States**:
- `INITIALIZING` (automatic)

---

### 2. INITIALIZING

**Purpose**: Initialize all hardware components

**Entry Conditions**:
- Came from START state

**Actions**:
1. Initialize Dynamixel motors
   - Open serial port
   - Ping all motors (IDs: 1, 2, 3)
   - Enable torque
   - Read neutral positions
2. Initialize trakSTAR tracking
   - Initialize BIRD system
   - Validate all 4 sensors
   - Perform initial position reads
3. Initialize NI-DAQ
   - Create analog input task (3 channels)
   - Configure sampling rate
   - Register callback
   - Start acquisition
4. Initialize shared memory
   - Create/open `Local\PyToCPP`
   - Create/open `Local\CPPToPy`

**Exit Conditions**:
- Success: All hardware initialized → CALIBRATION
- Failure: Any init fails → ERR

**Next States**:
- `CALIBRATION` (on success)
- `ERR` (on failure)

**Error Handling**:
- Log which component failed
- Set error message
- Transition to ERR state

---

### 3. CALIBRATION

**Purpose**: Automatically detect base positions and calculate workspace centroid

**Entry Conditions**:
- Came from INITIALIZING
- All hardware operational

**Actions**:

**Phase 1: Move to Base 0 (Left)**
1. Incrementally extend Motor 0 cable
2. Monitor Motor 0 position for stability
3. Detect arrival at base (position stabilizes)
4. Read and store Base 0 position from trakSTAR Sensor 0

**Phase 2: Move to Base 1 (Center)**
1. Retract Motor 0, extend Motor 1
2. Monitor Motor 1 position for stability
3. Detect arrival at base
4. Read and store Base 1 position from trakSTAR Sensor 1

**Phase 3: Move to Base 2 (Right)**
1. Retract Motor 1, extend Motor 2
2. Monitor Motor 2 position for stability
3. Detect arrival at base
4. Read and store Base 2 position from trakSTAR Sensor 2

**Phase 4: Calculate Centroid**
1. Compute centroid: `(Base0 + Base1 + Base2) / 3`
2. Validate triangle formed by bases
3. Set motors to centroid position

**Phase 5: Validate**
1. Move to centroid
2. Verify trakSTAR Sensor 3 reads correct position
3. Store static base positions for reference

**Stability Detection**:
```cpp
// Position is stable if it doesn't change significantly
// over N consecutive reads
bool detectBaseArrival(int motorIndex) {
    int stableCount = 0;
    int prevPosition = 0;

    while (stableCount < config.calibrationStabilityReads) {
        int currentPosition = motors[motorIndex].readCurrentPosition();

        if (abs(currentPosition - prevPosition) <
            config.calibrationPositionTolerance) {
            stableCount++;
        } else {
            stableCount = 0;
        }

        prevPosition = currentPosition;
    }

    return true;
}
```

**Exit Conditions**:
- Success: All bases detected → READY
- Failure: Sensor errors, timeout → ERR

**Next States**:
- `READY` (on success)
- `ERR` (on failure)

**Typical Duration**: 30-60 seconds

---

### 4. READY

**Purpose**: Idle state, waiting for commands from Python GUI

**Entry Conditions**:
- Came from CALIBRATION (successful)
- OR came from RUNNING (no new commands)
- System at centroid or last target position

**Actions**:
1. Read shared memory for new commands
2. Update status to Python GUI
3. Monitor sensors (tracking, load cells)
4. Log periodic status updates

**Command Processing**:
```cpp
MotorCommand cmd;
if (readSharedMemoryCommand(cmd)) {
    if (cmd.exit) {
        // Shutdown requested
        return States::END;
    }

    if (cmd.execute) {
        // New target position
        desiredPosition.x = cmd.target_x;
        desiredPosition.y = cmd.target_y;
        desiredPosition.z = cmd.target_z;

        if (validateMotorCommand(cmd)) {
            return States::MOVING;
        } else {
            // Invalid position
            return States::ERR;
        }
    }
}
```

**Exit Conditions**:
- New command received → MOVING
- Exit command → END
- Error detected → ERR

**Next States**:
- `MOVING` (command received)
- `END` (shutdown requested)
- `ERR` (error detected)

---

### 5. MOVING

**Purpose**: Actively moving platform to target position

**Entry Conditions**:
- Came from READY with new target
- OR came from TENSION needing more movement

**Actions**:
1. Read current position from trakSTAR Sensor 3
2. Calculate motor destinations for target
3. Send goal positions to all 3 motors
4. Wait for motors to reach positions
5. Read new current position
6. Calculate position error
7. Check if within tolerance

**Motor Position Calculation**:
```cpp
void calculateMotorPositionsFromCommand(const MotorCommand& cmd) {
    DOUBLE_POSITION_ANGLES_RECORD targetPos;
    targetPos.x = cmd.target_x;
    targetPos.y = cmd.target_y;
    targetPos.z = cmd.target_z;

    setMotorDestinationsForTarget(targetPos);
}

void setMotorDestinationsForTarget(DOUBLE_POSITION_ANGLES_RECORD& targetPos) {
    // Calculate cable lengths from each base to target
    for (int i = 0; i < 3; i++) {
        double dx = targetPos.x - staticBasePositions[i].x;
        double dy = targetPos.y - staticBasePositions[i].y;
        double dz = targetPos.z - staticBasePositions[i].z;

        double cableLength = sqrt(dx*dx + dy*dy + dz*dz);

        // Convert cable length to motor position
        motorDestinations[i] = neutralPos - (int)(cableLength * STEPS_PER_MM);
    }
}
```

**Position Validation**:
```cpp
bool currentCloseToDesired() {
    double dx = currentPosition.x - desiredPosition.x;
    double dy = currentPosition.y - desiredPosition.y;
    double dz = currentPosition.z - desiredPosition.z;

    double errorMagnitude = sqrt(dx*dx + dy*dy + dz*dz);

    if (errorMagnitude < config.posErrorThreshold) {
        // Also check angle between error vector and plane normal
        double angle = calculateErrorAngle();
        return (angle < config.angleThreshold);
    }

    return false;
}
```

**Exit Conditions**:
- Position reached (within tolerance) → RUNNING
- Tension needs adjustment → TENSION
- Error detected → ERR
- Shutdown requested → END

**Next States**:
- `RUNNING` (position achieved)
- `TENSION` (cables need adjustment)
- `ERR` (error)
- `END` (shutdown)

---

### 6. RUNNING

**Purpose**: Position achieved, monitoring and maintaining position

**Entry Conditions**:
- Came from MOVING with position achieved
- Current position within tolerance of desired

**Actions**:
1. Monitor current position
2. Check if position drifts from target
3. Monitor cable tension (load cells)
4. Update status to Python GUI
5. Check for new commands

**Drift Detection**:
```cpp
if (!currentCloseToDesired()) {
    // Position drifted, need to adjust
    return States::MOVING;
}
```

**Tension Monitoring**:
```cpp
double averages[3];
if (getDAQChannelAverages(averages)) {
    for (int i = 0; i < 3; i++) {
        if (averages[i] < config.minLoadVoltage ||
            averages[i] > config.maxLoadVoltage) {
            // Tension out of range
            return States::TENSION;
        }
    }
}
```

**Exit Conditions**:
- Position drifted → MOVING
- Tension out of range → TENSION
- New command → MOVING
- Shutdown → END

**Next States**:
- `MOVING` (drift or new command)
- `TENSION` (cable adjustment needed)
- `READY` (stable, no commands)
- `END` (shutdown)

---

### 7. TENSION

**Purpose**: Adjust cable tension based on load cell readings

**Entry Conditions**:
- Came from RUNNING or MOVING
- Load cell voltage out of acceptable range

**Actions**:
1. Read load cell voltages for all 3 cables
2. Identify which cables need adjustment
3. Make small motor position adjustments
4. Wait and re-read tensions
5. Verify tensions are now acceptable

**Tension Adjustment Logic**:
```cpp
bool adjustTensionBasedOnLoadCells() {
    double averages[3];
    if (!getDAQChannelAverages(averages)) {
        return false;
    }

    bool adjusted = false;

    for (int i = 0; i < 3; i++) {
        if (averages[i] < config.minLoadVoltage) {
            // Too loose - retract cable (increase tension)
            int currentPos = motors[i].readCurrentPosition();
            motors[i].setGoalPosition(currentPos - config.tensionAdjustmentSteps);
            adjusted = true;
        } else if (averages[i] > config.maxLoadVoltage) {
            // Too tight - extend cable (decrease tension)
            int currentPos = motors[i].readCurrentPosition();
            motors[i].setGoalPosition(currentPos + config.tensionAdjustmentSteps);
            adjusted = true;
        }
    }

    return adjusted;
}
```

**Exit Conditions**:
- Tensions corrected → RUNNING
- Position changed significantly → MOVING
- Correction failed → ERR

**Next States**:
- `RUNNING` (tensions good)
- `MOVING` (position affected)
- `ERR` (adjustment failed)

**Typical Duration**: < 1 second

---

### 8. ERR (Error)

**Purpose**: Handle errors and attempt recovery

**Entry Conditions**:
- Hardware initialization failed
- Safety check failed
- Communication error
- Sensor error
- Any exceptional condition

**Actions**:
1. Log error details
2. Disable motor torque (safety)
3. Stop DAQ acquisition (optional)
4. Update error status to Python GUI
5. Wait for manual intervention or timeout

**Error Recovery**:
```cpp
// Minimal recovery - disable motors for safety
bool disableMotors() {
    bool allDisabled = true;
    for (auto& motor : motors) {
        if (!motor.disableTorque()) {
            allDisabled = false;
        }
    }
    return allDisabled;
}
```

**Exit Conditions**:
- Manual reset (restart application)
- Automatic: Transition to CLEANUP after timeout

**Next States**:
- `END` (shutdown requested)
- `CLEANUP` (timeout)

**Notes**:
- System requires restart after entering ERR state
- No automatic recovery to prevent unsafe operation

---

### 9. END

**Purpose**: Prepare for controlled shutdown

**Entry Conditions**:
- User requested shutdown (via GUI or Ctrl+C)
- Fatal error requiring termination

**Actions**:
1. Disable motor torque
2. Stop tracking (optional - maintain position data)
3. Stop DAQ acquisition
4. Close shared memory connections
5. Flush logs

**Exit Conditions**:
- Always transitions to CLEANUP

**Next States**:
- `CLEANUP` (automatic)

---

### 10. CLEANUP

**Purpose**: Final resource cleanup before exit

**Entry Conditions**:
- Came from END state

**Actions**:
1. Close Dynamixel serial port
2. Close trakSTAR system
3. Clear NI-DAQ tasks
4. Unmap shared memory
5. Close log files

**Exit Conditions**:
- Cleanup complete → Exit program

**Next States**:
- None (program terminates)

---

## State Transitions

### Transition Table

| From State     | To State       | Trigger Condition                          |
|----------------|----------------|--------------------------------------------|
| START          | INITIALIZING   | Automatic                                  |
| INITIALIZING   | CALIBRATION    | All hardware initialized successfully      |
| INITIALIZING   | ERR            | Hardware initialization failed             |
| CALIBRATION    | READY          | Calibration successful                     |
| CALIBRATION    | ERR            | Calibration failed                         |
| READY          | MOVING         | New target command received                |
| READY          | END            | Shutdown requested                         |
| READY          | ERR            | Error detected                             |
| MOVING         | RUNNING        | Position achieved (within tolerance)       |
| MOVING         | TENSION        | Tension adjustment needed                  |
| MOVING         | ERR            | Error during movement                      |
| MOVING         | END            | Shutdown requested                         |
| RUNNING        | MOVING         | Position drifted or new command            |
| RUNNING        | TENSION        | Tension out of range                       |
| RUNNING        | READY          | Stable, no commands (optional)             |
| RUNNING        | END            | Shutdown requested                         |
| TENSION        | RUNNING        | Tensions corrected                         |
| TENSION        | MOVING         | Position changed during adjustment         |
| TENSION        | ERR            | Adjustment failed                          |
| ERR            | END            | Shutdown requested                         |
| ERR            | CLEANUP        | Timeout                                    |
| END            | CLEANUP        | Automatic                                  |
| CLEANUP        | (Exit)         | Automatic                                  |

### Transition Logic (Simplified)

```cpp
States SystemController::processStateTransition(States current) {
    switch (current) {
        case States::START:
            return States::INITIALIZING;

        case States::INITIALIZING:
            if (initializeHardware()) {
                return States::CALIBRATION;
            } else {
                return States::ERR;
            }

        case States::CALIBRATION:
            if (performCalibration()) {
                return States::READY;
            } else {
                return States::ERR;
            }

        case States::READY: {
            MotorCommand cmd;
            if (readSharedMemoryCommand(cmd)) {
                if (cmd.exit) return States::END;
                if (cmd.execute && validateMotorCommand(cmd)) {
                    return States::MOVING;
                }
            }
            return States::READY;  // Stay in READY
        }

        case States::MOVING:
            if (currentCloseToDesired()) {
                return States::RUNNING;
            }
            // Check if tension adjustment needed
            if (tensionOutOfRange()) {
                return States::TENSION;
            }
            return States::MOVING;  // Continue moving

        case States::RUNNING:
            if (!currentCloseToDesired()) {
                return States::MOVING;
            }
            if (tensionOutOfRange()) {
                return States::TENSION;
            }
            // Check for new commands
            return States::RUNNING;  // Stay in RUNNING

        case States::TENSION:
            if (adjustTensionBasedOnLoadCells()) {
                return States::RUNNING;
            } else {
                return States::ERR;
            }

        case States::ERR:
            disableMotors();
            return States::END;  // Require shutdown

        case States::END:
            return States::CLEANUP;

        case States::CLEANUP:
            // Cleanup and exit program
            return States::CLEANUP;

        default:
            return States::ERR;
    }
}
```

---

## State Diagrams

### High-Level Flow

```
           START
             │
             ↓
       INITIALIZING ─────→ ERR
             │               │
             ↓               ↓
        CALIBRATION         END
             │               │
             ↓               ↓
           READY          CLEANUP
             │               │
             ↓               ↓
           MOVING          (Exit)
           ↙    ↘
    RUNNING    TENSION
       ↓    ↙       ↘
       └──→   ←─────┘
```

### Detailed Normal Operation Flow

```
READY
  │
  │ [New Command]
  ↓
MOVING
  │
  │ [Position Check]
  ↓
  ├─[In Tolerance]─→ RUNNING
  │                    │
  │                    │ [Monitor]
  │                    ↓
  │                    ├─[Drift Detected]───→ MOVING
  │                    │
  │                    ├─[Tension Issue]────→ TENSION
  │                    │                        │
  │                    │                        │ [Adjust]
  │                    │                        ↓
  │                    │                      RUNNING
  │                    │
  │                    └─[New Command]──────→ MOVING
  │
  └─[Out of Tolerance]─→ Continue MOVING or TENSION
```

### Error Handling Flow

```
Any State
    │
    │ [Error Detected]
    ↓
   ERR
    │
    │ [Disable Motors]
    ↓
   END
    │
    ↓
 CLEANUP
    │
    ↓
  (Exit)
```

---

## Implementation Details

### Main Control Loop

```cpp
void SystemController::run() {
    while (currentState != States::CLEANUP) {
        // Log state if changed
        if (currentState != previousState) {
            spdlog::info("State transition: {} → {}",
                         stateToString(previousState),
                         stateToString(currentState));
            previousState = currentState;
        }

        // Process current state and determine next
        States nextState = processStateTransition(currentState);

        // Update state
        currentState = nextState;

        // Update shared memory status
        sharedMemory->writeStatus(getCurrentStatusUpdate());

        // Small delay to prevent CPU spinning
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // Final cleanup
    shutdown();
}
```

### State Persistence

States are logged to file for debugging:
```
[2024-12-01 13:45:23.001] [I]: State: START
[2024-12-01 13:45:23.015] [I]: State: INITIALIZING
[2024-12-01 13:45:24.532] [I]: State: CALIBRATION
[2024-12-01 13:45:55.123] [I]: State: READY
[2024-12-01 13:46:10.456] [I]: State: MOVING
[2024-12-01 13:46:12.789] [I]: State: RUNNING
```

### Thread Safety

State transitions occur only in the main thread. The DAQ callback thread does not modify the state machine, only updates shared data (load cell readings).

---

## Timing Considerations

### State Duration (Typical)

| State         | Typical Duration  | Notes                              |
|---------------|-------------------|------------------------------------|
| START         | < 1ms             | Instantaneous                      |
| INITIALIZING  | 2-5 seconds       | Hardware dependent                 |
| CALIBRATION   | 30-60 seconds     | Automatic base detection           |
| READY         | Variable          | Until command received             |
| MOVING        | 1-10 seconds      | Distance dependent                 |
| RUNNING       | Variable          | Until drift or new command         |
| TENSION       | < 1 second        | Small adjustments                  |
| ERR           | Until restart     | Manual intervention required       |
| END           | < 100ms           | Quick shutdown prep                |
| CLEANUP       | < 1 second        | Resource cleanup                   |

### Control Loop Frequency

- **Target**: 10-20 Hz (50-100ms per iteration)
- **Typical**: 15 Hz (~67ms per iteration)
- **Limiting Factor**: trakSTAR position reads (~10ms per sensor)

---

## Debugging Tips

### Enable State Transition Logging

All state transitions are logged by default. To see detailed debug info:
```cpp
spdlog::set_level(spdlog::level::debug);
```

### Monitor State Changes in GUI

Python GUI displays current state in status bar. Watch for unexpected transitions.

### Common State Issues

1. **Stuck in MOVING**:
   - Position tolerance too tight
   - Motors not reaching targets
   - Tracking sensor noise

2. **Frequent TENSION transitions**:
   - Load cell calibration needed
   - Voltage thresholds too narrow
   - Cable friction issues

3. **Immediate ERR after INITIALIZING**:
   - Hardware not connected
   - Driver not installed
   - Permission issues

---

## Related Documentation

- [Main README](../README.md)
- [Architecture Documentation](ARCHITECTURE.md)
- [Build Guide](BUILD_GUIDE.md)
- [Hardware Setup](HARDWARE_SETUP.md)
