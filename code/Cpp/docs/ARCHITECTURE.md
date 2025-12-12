# Heartprinter C++ System Architecture

This document describes the software architecture of the Heartprinter C++ control system.

---

## Table of Contents

1. [Overview](#overview)
2. [Component Architecture](#component-architecture)
3. [Class Structure](#class-structure)
4. [Data Flow](#data-flow)
5. [Thread Model](#thread-model)
6. [Communication Protocols](#communication-protocols)
7. [Error Handling](#error-handling)

---

## Overview

The Heartprinter C++ system uses a **layered architecture** with a **state machine core** for control flow:

```
┌──────────────────────────────────────────────────┐
│                Application Layer                  │
│              (SystemController)                   │
└────────────────────┬─────────────────────────────┘
                     │
┌────────────────────┴─────────────────────────────┐
│             Hardware Abstraction Layer            │
│  (Motor, CTrackstar, NIDAQ, SharedMemoryManager) │
└────────────────────┬─────────────────────────────┘
                     │
┌────────────────────┴─────────────────────────────┐
│               Driver/SDK Layer                    │
│    (Dynamixel SDK, trakSTAR API, NI-DAQmx)      │
└──────────────────────────────────────────────────┘
```

### Design Principles

1. **Separation of Concerns**: Each hardware component has its own abstraction class
2. **State Machine Control**: Robust state-based control flow
3. **RAII**: Resource Acquisition Is Initialization for automatic cleanup
4. **Exception Safety**: All hardware operations can throw exceptions
5. **Thread Safety**: Shared resources protected by mutexes
6. **Logging**: Comprehensive logging at all levels

---

## Component Architecture

### 1. SystemController (Main Controller)

**File**: `include/custom/SystemController.h`, `src/custom/SystemController.cpp`

**Purpose**: Central coordinator that manages all hardware components and implements the state machine.

**Key Responsibilities**:
- State machine execution
- Hardware initialization and cleanup
- Command processing from Python GUI
- Position control logic
- Safety validation
- Logging coordination

**Key Methods**:
```cpp
bool initialize();                    // Initialize all hardware
void run();                          // Main control loop (blocking)
void shutdown();                     // Clean shutdown
States processStateTransition();     // State machine logic
bool performSafetyCheck();          // Validate target positions
```

**State Machine States**:
- `START` - Initial state
- `INITIALIZING` - Hardware setup
- `CALIBRATION` - Base detection
- `READY` - Idle, waiting for commands
- `RUNNING` - Position achieved
- `MOVING` - Active movement
- `TENSION` - Tension adjustment
- `ERROR` - Error recovery
- `END` - Shutdown requested
- `CLEANUP` - Resource cleanup

### 2. Motor Class

**File**: `include/custom/Motor.h`, `src/custom/Motor.cpp`

**Purpose**: Controls individual Dynamixel motors via TTL communication.

**Key Responsibilities**:
- Motor initialization and configuration
- Position reading and writing
- Velocity control
- Torque enabling/disabling
- Error detection

**Key Methods**:
```cpp
bool init(int id, PortHandler* port, PacketHandler* packet);
bool setGoalPosition(int position);
int readCurrentPosition();
bool enableTorque();
bool disableTorque();
```

**Motor Configuration**:
- Protocol: Dynamixel Protocol 2.0
- Control Mode: Position control (extended position mode)
- Communication: TTL (3-wire, Half Duplex UART) via U2D2 or USB2Dynamixel
- Baud Rate: 57600

### 3. CTrackstar Class (Position Tracking)

**File**: `include/custom/CTrackstar.h`, `src/custom/CTrackstar.cpp`

**Purpose**: Interfaces with Northern Digital trakSTAR electromagnetic tracking system.

**Key Responsibilities**:
- trakSTAR initialization
- Sensor configuration
- Real-time position/orientation reading
- Sensor validation
- Error handling

**Key Methods**:
```cpp
bool initialize();
bool getSingleRecord(int sensorId, DOUBLE_POSITION_ANGLES_RECORD& record);
bool validateSensor(int sensorId);
void cleanup();
```

**Tracking Data Structure**:
```cpp
struct DOUBLE_POSITION_ANGLES_RECORD {
    double x, y, z;          // Position in mm
    double azimuth;          // Rotation angles
    double elevation;
    double roll;
    double quality;          // Signal quality
    int time;               // Timestamp
};
```

### 4. NIDAQ Module (Load Cell Monitoring)

**File**: `include/custom/NIDAQ.h`, `src/custom/NIDAQ.cpp`

**Purpose**: Interfaces with NI-DAQ hardware for load cell voltage monitoring.

**Key Responsibilities**:
- DAQ device initialization
- Analog input configuration (3 channels)
- Continuous data acquisition
- Real-time averaging
- Callback handling for asynchronous data

**Key Structures**:
```cpp
struct DAQSystem {
    TaskHandle analogTaskHandle;
    TaskHandle digitalTaskHandle;
    bool analogActive;
    bool digitalActive;
};

struct AnalogConfig {
    std::string deviceName;
    std::string channelNames;
    double minVoltage;
    double maxVoltage;
    int samplesPerChannel;
    double sampleRate;
};
```

**Callback Architecture**:
```cpp
int32 CVICALLBACK AnalogCallback(
    TaskHandle taskHandle,
    int32 everyNsamplesEventType,
    uInt32 nSamples,
    void *callbackData
);
```

### 5. SharedMemoryManager (IPC with Python)

**File**: `include/custom/SharedMemoryManager.h`, `src/custom/SharedMemoryManager.cpp`

**Purpose**: Manages shared memory communication between C++ and Python GUI.

**Key Responsibilities**:
- Shared memory creation/opening
- Reading commands from Python
- Writing status updates to Python
- Thread-safe access
- Memory cleanup

**Shared Memory Structures**:
```cpp
// Python → C++ (Commands)
struct MotorCommand {
    double target_x;
    double target_y;
    double target_z;
    bool execute;          // Execute command flag
    bool exit;            // Exit system flag
};

// C++ → Python (Status)
struct StatusUpdate {
    double baseL[3];      // Left base position
    double baseC[3];      // Center base position
    double baseR[3];      // Right base position
    double baseMP[3];     // Moving platform position
    char status[6];       // Current state string
};
```

**Memory Names**:
- `Local\PyToCPP` - Commands from Python
- `Local\CPPToPy` - Status to Python

---

## Class Structure

### Inheritance Hierarchy

```
SystemController (main controller)
├── Uses: Motor (composition)
├── Uses: CSystem (unique_ptr)
├── Uses: DAQSystem (unique_ptr)
└── Uses: SharedMemoryManager (unique_ptr)

Motor (individual motor control)
├── Uses: dynamixel::PortHandler*
└── Uses: dynamixel::PacketHandler*

CTrackstar (tracking system)
└── Uses: trakSTAR API (ATC3DG.h)

NIDAQ (load cell monitoring)
└── Uses: NI-DAQmx API
```

### Key Design Patterns

1. **RAII (Resource Acquisition Is Initialization)**
   - Resources acquired in constructor
   - Resources released in destructor
   - Ensures cleanup even on exceptions

2. **Singleton-like** (SystemController)
   - Only one instance should exist
   - Non-copyable, non-movable

3. **State Pattern** (State Machine)
   - Each state has defined entry/exit logic
   - Transitions controlled by `processStateTransition()`

4. **Observer Pattern** (DAQ Callbacks)
   - Asynchronous data notifications
   - Callback functions registered with DAQ

---

## Data Flow

### Control Loop Flow

```
┌─────────────────────────────────────────────────┐
│                   run()                          │ ← Main Loop
└──────────────┬──────────────────────────────────┘
               │
               ↓
┌──────────────────────────────────────────────────┐
│         processStateTransition()                 │ ← State Machine
└──────────────┬───────────────────────────────────┘
               │
               ↓
┌──────────────┴───────────────────────────────────┐
│  Current State Actions:                          │
│  - Read sensors (tracking, load cells)           │
│  - Process commands (shared memory)              │
│  - Calculate motor positions                     │
│  - Execute movements                             │
│  - Update status                                 │
└──────────────┬───────────────────────────────────┘
               │
               ↓
        Next State Selection
               │
               └──> Loop
```

### Position Control Flow

```
Python GUI                    C++ System                   Hardware
    │                             │                           │
    │─────[Set Target]──────────>│                           │
    │                             │                           │
    │                             │────[Validate Position]────│
    │                             │                           │
    │                             │────[Calculate Motors]─────│
    │                             │                           │
    │                             │────[Move Motors]────────>│
    │                             │                           │
    │<────[Status Update]─────────│                           │
    │                             │                           │
    │                             │<────[Read Position]───────│
    │                             │                           │
    │                             │<────[Read Load Cells]─────│
    │                             │                           │
    │                             │────[Adjust Tension]─────>│
    │                             │                           │
    │<────[Status: RUNNING]───────│                           │
```

### Data Structures Flow

```
Command Input:
Python → SharedMemory → MotorCommand → SystemController

Position Data:
trakSTAR → CTrackstar → DOUBLE_POSITION_ANGLES_RECORD → SystemController

Load Cell Data:
NI-DAQ → Callback → DAQSystem → daqChannelAverages[] → SystemController

Motor Control:
SystemController → motorDestinations[] → Motor → Dynamixel SDK → Hardware

Status Output:
SystemController → StatusUpdate → SharedMemory → Python
```

---

## Thread Model

### Main Thread (SystemController::run())

**Responsibilities**:
- State machine execution
- Command processing
- Motor control
- Position tracking
- Logging

**Blocking**: Yes - runs until shutdown requested

### DAQ Callback Thread (NI-DAQmx)

**Responsibilities**:
- Asynchronous data acquisition
- Real-time averaging of load cell voltages
- Updates `daqChannelAverages[]` array

**Thread Safety**:
- Mutex: `daqDataMutex`
- Atomic flag: `daqDataAvailable`

**Callback Frequency**: ~100 Hz (configurable via `samplesPerChannel`)

### Thread Safety Mechanisms

```cpp
class SystemController {
    // Shared between main thread and DAQ callback
    mutable std::mutex daqDataMutex;
    std::atomic<bool> daqDataAvailable{false};
    double daqChannelAverages[3];
};

// Safe read from main thread:
bool getDAQChannelAverages(double averages[3]) const {
    std::lock_guard<std::mutex> lock(daqDataMutex);
    // Copy data
    return daqDataAvailable.load();
}

// Safe write from callback thread:
int32 CVICALLBACK AnalogCallback(...) {
    std::lock_guard<std::mutex> lock(controller->daqDataMutex);
    // Update averages
    controller->daqDataAvailable.store(true);
}
```

---

## Communication Protocols

### 1. Dynamixel Communication (TTL)

**Protocol**: Dynamixel Protocol 2.0
**Interface**: TTL (3-wire: GND, Data, VCC)
**Logic Level**: 0-5V TTL
**Baud Rate**: 57600
**Packet Structure**:
```
[0xFF][0xFF][0xFD][0x00][ID][LEN_L][LEN_H][INST][PARAM...][CRC_L][CRC_H]
```

**Key Operations**:
- Write Goal Position (Address: 116, Size: 4 bytes)
- Read Present Position (Address: 132, Size: 4 bytes)
- Enable/Disable Torque (Address: 64, Size: 1 byte)

### 2. trakSTAR Communication

**Interface**: USB/Serial via driveBAY
**API**: ATC3DG.h (proprietary)
**Data Rate**: Up to 420 Hz per sensor

**Key Functions**:
```cpp
InitializeBIRDSystem();
GetSynchronousRecord(sensorId, &record, sizeof(record));
CloseBIRDSystem();
```

### 3. NI-DAQmx Communication

**Interface**: USB/PCI
**Mode**: Continuous analog input
**Channels**: 3 differential inputs (CH0, CH1, CH2)
**Sample Rate**: Configurable (default: 1000 Hz)

**Configuration**:
```cpp
DAQmxCreateTask("AnalogTask", &taskHandle);
DAQmxCreateAIVoltageChan(task, "Dev1/ai0:2", "",
                         DAQmx_Val_Cfg_Default,
                         minVolt, maxVolt,
                         DAQmx_Val_Volts, NULL);
DAQmxCfgSampClkTiming(task, "", sampleRate, ...);
DAQmxRegisterEveryNSamplesEvent(task, ..., callback, ...);
DAQmxStartTask(task);
```

### 4. Shared Memory IPC

**Platform**: Windows Named Shared Memory
**Access**: Read/Write with mutex protection
**Update Rate**:
- Commands (Python→C++): On demand
- Status (C++→Python): ~10 Hz

---

## Error Handling

### Exception Hierarchy

```cpp
std::runtime_error
    └── SystemException
            ├── MOTOR_INIT_FAILED
            ├── TRACKER_INIT_FAILED
            ├── DAQ_INIT_FAILED
            ├── COMMUNICATION_FAILED
            ├── SAFETY_VIOLATION
            └── EMERGENCY_STOP
```

### Error Handling Strategy

1. **Hardware Initialization**:
   - Try-catch blocks around all init calls
   - Throw `SystemException` on failure
   - Cleanup resources before throwing

2. **Runtime Errors**:
   - State transitions to `ERROR` state
   - Log error details
   - Attempt recovery when possible
   - Fallback: Disable motors and request shutdown

3. **Safety Violations**:
   - Immediate motor disable
   - Transition to `ERROR` state
   - Require manual restart

### Logging Strategy

```cpp
// spdlog levels used:
spdlog::info("Status message");       // Normal operation
spdlog::warn("Warning condition");    // Potential issues
spdlog::error("Error occurred");      // Errors
spdlog::critical("Critical failure"); // Fatal errors
spdlog::debug("Debug info");          // Detailed debugging
```

**Log File Location**: `logs/log_YYYYMMDD_HHMMSS.txt`

**Log Format**:
```
[2024-12-01 13:45:23.123] [I]: System initialized successfully
[2024-12-01 13:45:24.456] [W]: Motor 0 position error: 5 steps
[2024-12-01 13:45:25.789] [E]: Tracking sensor 3 quality low: 0.45
```

---

## Performance Considerations

### Control Loop Timing

- **Target**: < 100ms per iteration
- **Typical**: 20-50ms per iteration
- **Bottleneck**: trakSTAR position reads (~10ms each)

### Optimization Points

1. **Batch Motor Operations**: Use GroupSyncRead/Write
2. **Async DAQ**: Callbacks don't block main loop
3. **Selective Logging**: Debug logs only when enabled
4. **Efficient State Transitions**: Minimal computation per state

### Memory Usage

- **Static Allocation**: Prefer stack over heap
- **Smart Pointers**: unique_ptr for hardware components
- **Minimal Copying**: Pass by reference where possible
- **Shared Memory**: Fixed-size structures (no dynamic allocation)

---

## Future Improvements

### Potential Enhancements

1. **Configuration Files**: Load settings from config file instead of hardcoding
2. **Command Line Arguments**: Support runtime configuration
3. **Plugin Architecture**: Modular hardware interfaces
4. **Network Communication**: TCP/IP in addition to shared memory
5. **Real-time OS**: Deterministic timing with RTOS
6. **PID Controllers**: Advanced motion control algorithms
7. **Trajectory Planning**: Smooth path interpolation

---

## Related Documentation

- [Main README](../README.md)
- [State Machine Documentation](STATE_MACHINE.md)
- [Build Guide](BUILD_GUIDE.md)
- [Hardware Setup](HARDWARE_SETUP.md)
