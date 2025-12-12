# Heartprinter Hardware Setup Guide

Complete guide for setting up and connecting all hardware components of the Heartprinter system.

---

## Table of Contents

1. [Safety First](#safety-first)
2. [Hardware Overview](#hardware-overview)
3. [Component Setup](#component-setup)
4. [Wiring Diagram](#wiring-diagram)
5. [Motor Configuration](#motor-configuration)
6. [Tracking System Setup](#tracking-system-setup)
7. [Load Cell Setup](#load-cell-setup)
8. [Testing Individual Components](#testing-individual-components)
9. [System Integration](#system-integration)
10. [Troubleshooting](#troubleshooting)

---

## Safety First

### ⚠️ Important Safety Guidelines

Before working with the hardware:

1. **Power Off**: Disconnect all power before making connections
2. **Verify Voltage**: Ensure power supplies match component specifications
3. **No Shorts**: Check for short circuits before powering on
4. **Cable Management**: Keep cables organized to prevent tangling during operation
5. **Emergency Stop**: Know how to quickly power off the system (Ctrl+C or GUI End button)
6. **Workspace Clear**: Keep the operational area clear of obstacles
7. **Supervision**: Never leave the system running unattended
8. **Load Limits**: Do not exceed motor stall torque (1.4 N⋅m)

### Power Requirements

| Component           | Voltage    | Current    | Power Supply           |
|---------------------|------------|------------|------------------------|
| Dynamixel Motors (3)| 12V        | 2.3A peak  | 12V 5A power supply    |
| trakSTAR driveBAY   | Via USB    | USB power  | Computer USB or hub    |
| NI-DAQ Device       | Via USB/PCI| Varies     | Computer USB or PCI    |
| Load Cells (3)      | 5-10V      | <50mA each | Included in DAQ        |

---

## Hardware Overview

### Bill of Materials

#### Required Components

**1. Motion Control**
- 3x [Dynamixel XC430-T150BB-T](https://www.robotis.us/dynamixel-xc430-t150bb-t/) motors
- 1x U2D2 or USB2Dynamixel adapter
- 1x 12V 5A power supply for motors
- 3x Motor cables (included with motors)
- 1x Power hub (optional, for daisy-chaining)

**2. Position Tracking**
- 1x Northern Digital trakSTAR system
- 1x driveBAY interface unit
- 4x Electromagnetic sensors (3 bases + 1 moving platform)
- Sensor cables

**3. Load Monitoring**
- 1x National Instruments DAQ device (USB or PCI)
  - Minimum 3 analog input channels
  - Examples: USB-6001, USB-6003, PCIe-6321
- 3x Load cells (analog voltage output)
- Wiring for analog connections

**4. Mechanical (Not covered in detail)**
- Cable-driven parallel robot frame
- Mounting brackets for motors
- Cable routing system
- Platform attachment points

**5. Computer**
- Windows 10/11 64-bit or Linux
- USB ports: 2-3 available
- Minimum 4GB RAM, quad-core processor

#### Optional Components
- USB hub (powered, if USB ports are limited)
- Cable management clips
- Multimeter (for testing)
- Logic analyzer (for debugging TTL communication)

---

## Component Setup

### 1. Dynamixel Motor Setup

#### Motor Specifications

**Dynamixel XC430-T150BB-T**
- **Operating Voltage**: 9-12V (Recommended: 12V)
- **Stall Torque**: 1.4 N⋅m @ 12V
- **No Load Speed**: 46 RPM @ 12V
- **Resolution**: 4096 units per revolution
- **Protocol**: Dynamixel Protocol 2.0
- **Communication**: TTL (3-wire, Half-Duplex UART, 0-5V logic)
- **Baud Rate**: 57600 (configurable)
- **Default ID**: 1 (must be changed for motors 2 and 3)

#### Physical Installation

**1. Mount Motors**
```
- Mount each motor securely to the frame
- Motor 0: Base Left position
- Motor 1: Base Center position
- Motor 2: Base Right position
- Ensure motors can rotate freely
- Verify mounting screws are tight (M2.5)
```

**2. Attach Cables to Motors**
```
- Cable should spool around motor horn
- Use fishing line, Kevlar thread, or steel cable
- Typical cable length: 1-2 meters per motor
- Attach load cell inline with cable
```

#### Electrical Connections

**Daisy-Chain Configuration** (Recommended)

```
[12V Power Supply]
       │
       ├─[+]─┐
       └─[-]─┼──> [Power Hub]
             │         │
             │    ┌────┼────┬────────┐
             │    │    │    │        │
          [U2D2]  │    │    │        │
             │    ↓    ↓    ↓        │
         [USB]─> PC  Motor 0  Motor 1  Motor 2
                      ID=1    ID=2    ID=3
```

**Important Note on Communication Protocol**:
The XC430-T150BB-T uses **TTL communication** (3-wire), not RS-485. Ensure your U2D2 or USB2Dynamixel adapter is configured for TTL mode.

**Step-by-Step Wiring**

1. **Connect U2D2 to Computer**
   ```
   - Plug U2D2 into available USB port
   - Ensure U2D2 is set to TTL mode (check switch if available)
   - Install FTDI driver if needed (usually automatic)
   - Note COM port (Windows) or /dev/ttyUSB0 (Linux)
   ```

2. **Connect Power Supply**
   ```
   - DO NOT power on yet
   - Connect 12V power supply to power hub
   - Verify polarity: Red = +12V, Black = GND
   ```

3. **Connect First Motor**
   ```
   - Connect JST cable from U2D2 to Motor 0
   - Note: 3-pin connector (GND, DATA, VCC)
   - Connection order doesn't matter (daisy-chain)
   ```

4. **Daisy-Chain Remaining Motors**
   ```
   - Motor 0 → Motor 1 (using JST cable)
   - Motor 1 → Motor 2 (using JST cable)
   - Each motor has two JST ports (IN and OUT)
   ```

5. **Power Connection**
   ```
   - Connect power hub to each motor's power port
   - Or use motors' power passthrough
   ```

**Alternative: Star Configuration**
```
If using individual power connections:
- Each motor gets its own power cable from hub
- All motors still daisy-chained for communication
```

#### Verify Connections

**Before Powering On**:
- [ ] All communication cables connected
- [ ] Power cables connected with correct polarity
- [ ] No loose connections
- [ ] U2D2 connected to computer

**Power On Sequence**:
1. Connect computer (U2D2 via USB)
2. Power on 12V supply
3. Motors should flash LED briefly
4. LED will be red (torque disabled) or green (torque enabled)

---

### 2. Motor ID Configuration

**Important**: Each motor must have a unique ID (1, 2, 3)

#### Using Dynamixel Wizard 2.0

**1. Download and Install**
```
Download: https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/
Install on Windows or Linux
```

**2. Connect Single Motor**
```
- For initial setup, connect ONLY ONE motor
- This avoids ID conflicts
```

**3. Scan for Motor**
```
1. Open Dynamixel Wizard 2.0
2. Select COM port (or /dev/ttyUSB0)
3. Select Protocol 2.0
4. Click "Scan" (test baud rates: 57600)
5. Motor should appear (default ID: 1)
```

**4. Change Motor Settings**

**Motor 0 (Base Left)**
```
ID: 1
Baud Rate: 57600
Operating Mode: Extended Position Control Mode (4)
```

**Motor 1 (Base Center)**
```
ID: 2
Baud Rate: 57600
Operating Mode: Extended Position Control Mode (4)
```

**Motor 2 (Base Right)**
```
ID: 3
Baud Rate: 57600
Operating Mode: Extended Position Control Mode (4)
```

**5. Configure Each Motor**
```
For each motor (one at a time):
1. Connect motor alone
2. Scan and find (ID will be default or previous)
3. Change ID to target ID (1, 2, or 3)
4. Set baud rate to 57600
5. Set Operating Mode to 4 (Extended Position Control)
6. Click "Save" to write to EEPROM
7. Disconnect and repeat for next motor
```

**6. Verify All Three**
```
- Connect all three motors in daisy-chain
- Scan with Dynamixel Wizard
- Should find IDs: 1, 2, 3
- Verify baud rate: 57600 for all
```

#### Register Configuration

**Critical Registers** (configured by software, but verify):

| Register Name       | Address | Value | Description              |
|---------------------|---------|-------|--------------------------|
| ID                  | 7       | 1-3   | Motor ID                 |
| Baud Rate           | 8       | 1     | 57600 bps                |
| Operating Mode      | 11      | 4     | Extended Position        |
| Torque Enable       | 64      | 0/1   | 0=Free, 1=Hold           |
| Goal Position       | 116     | varies| Target position (4 bytes)|
| Present Position    | 132     | varies| Current position (4 bytes)|

---

### 3. trakSTAR Tracking System Setup

#### System Components

- **driveBAY**: Main controller unit
- **4x EM Sensors**: Small electromagnetic sensors
- **Sensor Cables**: Connect sensors to driveBAY

#### Physical Setup

**1. driveBAY Placement**
```
- Place driveBAY on stable surface near computer
- Keep away from large metal objects
- Avoid proximity to power supplies (EM interference)
- Distance from workspace: 0.5-1 meter
```

**2. Connect Sensors**
```
Sensor 0: Base Left (static base position)
Sensor 1: Base Center (static base position)
Sensor 2: Base Right (static base position)
Sensor 3: Moving Platform (tracks end-effector position)

- Connect each sensor to numbered port on driveBAY
- Ensure cables are not tangled
- Route cables to avoid motor movement paths
```

**3. Sensor Placement**

**Base Sensors (0, 1, 2)**: Static Reference Points
```
- Attach to motor mounting locations (bases of triangle)
- Should form approximately equilateral triangle
- Typical spacing: 200-400mm between bases
- Secure so they don't move during operation
- Orient sensors with consistent alignment
```

**Platform Sensor (3)**: Moving Tracker
```
- Attach to end-effector or platform
- Should be centered on platform
- Ensure cables have slack for full range of motion
- Sensor should not rotate significantly during operation
```

**4. Coordinate System**
```
The system uses trakSTAR's coordinate frame:
- X, Y, Z in millimeters
- Origin at driveBAY transmitter
- Right-handed coordinate system

Calibration routine will establish:
- Base triangle positions (from Sensors 0, 1, 2)
- Workspace centroid
- Valid movement envelope
```

#### Electrical Connections

**1. Power driveBAY**
```
- Connect to power outlet
- Power switch on back
- Wait for initialization (~5 seconds)
- LED indicators should show green
```

**2. Connect to Computer**
```
- USB connection from driveBAY to computer
- Driver should install automatically (Windows)
- Linux: May require permissions (add user to dialout group)
```

**3. Verify Connection**
```
Windows:
- Device Manager → Ports (COM & LPT)
- Should see: "Northern Digital driveBAY"

Linux:
- lsusb | grep "Northern Digital"
- Should show device
```

#### Testing trakSTAR

**Using NDI Diagnostic Tools**

```
1. Open NDI diagnostic software (included with API)
2. Initialize system
3. Check all 4 sensors:
   - Sensor 0: Should show XYZ position
   - Sensor 1: Should show XYZ position
   - Sensor 2: Should show XYZ position
   - Sensor 3: Should show XYZ position
4. Wave sensors around - verify positions change
5. Note quality indicator (should be > 0.8 typically)
```

**Troubleshooting Tracking**
- **Low Quality**: Metal interference, sensor too far from transmitter
- **No Position**: Sensor unplugged or faulty
- **Erratic Readings**: EM interference from power supply or motors

---

### 4. NI-DAQ and Load Cell Setup

#### NI-DAQ Device Setup

**Supported Devices** (examples)
- USB-6001 (Low cost, 8 AI channels)
- USB-6003 (8 AI channels, up to 100 kS/s)
- PCIe-6321 (16 AI channels, high performance)

**1. Connect DAQ to Computer**
```
USB Devices:
- Plug into USB port
- Driver installation (NI-DAQmx should already be installed)

PCI/PCIe Devices:
- Power down computer
- Install card in PCIe slot
- Boot and install drivers
```

**2. Verify in NI MAX**
```
Windows:
1. Start → National Instruments → NI MAX
2. Expand "Devices and Interfaces"
3. Should see your DAQ device (e.g., "Dev1")
4. Right-click → Test Panels
5. Try reading analog inputs - should see voltages
```

**Linux**:
```bash
# Check device
ni-daqmx-config --devices

# Should list your DAQ device
```

#### Load Cell Wiring

**Load Cell Specifications** (Typical)
- Output: 0-5V or 0-10V (analog voltage)
- Wiring: 3-wire (Signal, +Excitation, Ground)
- Mounting: Inline with cable between motor and platform

**3 Load Cells Configuration**
```
Load Cell 0: Motor 0 cable (Base Left)
Load Cell 1: Motor 1 cable (Base Center)
Load Cell 2: Motor 2 cable (Base Right)
```

**Wiring to NI-DAQ**

```
Load Cell 0:
  Signal → AI0 (Analog Input 0)
  +Excitation → +5V (from DAQ or external supply)
  Ground → AGND (Analog Ground)

Load Cell 1:
  Signal → AI1
  +Excitation → +5V
  Ground → AGND

Load Cell 2:
  Signal → AI2
  +Excitation → +5V
  Ground → AGND
```

**Differential vs Single-Ended**

```
Recommended: Differential (less noise)

Differential Wiring:
AI0+ (Load Cell 0 signal)
AI0- (AGND or dedicated - line)

Single-Ended Wiring:
AI0 (Load Cell 0 signal)
AGND (Common ground for all)
```

**Load Cell Calibration**

```
1. With no load (cable slack):
   - Read voltage (should be near 0V or offset)
   - Note this as "zero load voltage"

2. With known load (e.g., 1kg weight):
   - Read voltage
   - Calculate: Sensitivity = (V_load - V_zero) / Mass

3. Configure in software:
   - Voltage range: -10V to +10V (or appropriate)
   - Scaling factor: From calibration
```

#### Testing DAQ

**Using NI MAX Test Panels**

```
1. NI MAX → Devices → Dev1 → Test Panels
2. Select "Analog Input"
3. Select channels: ai0, ai1, ai2
4. Mode: Continuous
5. Click "Start"
6. Should see voltage readings from load cells
7. Gentle pull on cables - voltages should change
```

**Expected Voltages** (no load):
- Baseline: ~0V to ~2.5V (depends on load cell)
- With tension: Voltage increases
- Over-tension: Voltage may exceed threshold (configured in software)

---

## Wiring Diagram

### Complete System Wiring

```
┌─────────────────────────────────────────────────────────────┐
│                         Computer                             │
│  - Windows 10/11 or Linux                                    │
│  - NI-DAQmx Driver                                           │
│  - trakSTAR API                                              │
│  - Dynamixel SDK                                             │
└───┬──────────────────┬──────────────────┬──────────────────┘
    │                  │                  │
    │ USB              │ USB              │ USB/PCI
    │                  │                  │
┌───▼────────┐    ┌───▼──────────┐  ┌───▼──────────┐
│   U2D2     │    │  trakSTAR    │  │   NI-DAQ     │
│   (TTL)    │    │  driveBAY    │  │   Device     │
└───┬────────┘    └───┬──────────┘  └───┬──────────┘
    │                 │                 │
    │ Daisy-chain     │ 4x Sensors      │ 3x Analog In
    │                 │                 │
┌───▼──────────┐  ┌──▼───┬───┬───┐ ┌──▼───┬───┬──┐
│ Motor 0      │  │Sen 0 │ 1 │ 2 │ │LC 0  │ 1 │ 2│
│ (ID=1)       │  │Base  │Bas│Bas│ │Cable │Cab│Ca│
│              │  │Left  │Cen│Rgt│ │ 0    │ 1 │ 2│
│ Cable ───────┼──┼──────┴───┴───┼─┤      │   │  │
│ Spool        │  │              │ │      │   │  │
└──────────────┘  │              │ └──────┴───┴──┘
                  │  ┌───▼───┐   │
┌──────────────┐  │  │Sen 3  │   │
│ Motor 1      │  │  │Moving │   │
│ (ID=2)       │  │  │Platfrm│   │
│              │  │  └───────┘   │
│ Cable ───────┼──┼──────────────┤
│ Spool        │  │              │
└──────────────┘  │              │
                  │              │
┌──────────────┐  │              │
│ Motor 2      │  │              │
│ (ID=3)       │  │              │
│              │  │              │
│ Cable ───────┼──┴──────────────┘
│ Spool        │
└──────────────┘

┌──────────────┐
│   12V 5A     │
│Power Supply  │
│              │
└────┬──┬──┬──┘
     │  │  │
     ▼  ▼  ▼
   Mtr0 Mtr1 Mtr2
```

### Cable Routing

```
Platform
   ↑
   │ (Sensor 3 attached here)
   │
   ├──[Cable 0]──[Load Cell 0]──[Motor 0 Spool]──(Sensor 0)
   │
   ├──[Cable 1]──[Load Cell 1]──[Motor 1 Spool]──(Sensor 1)
   │
   └──[Cable 2]──[Load Cell 2]──[Motor 2 Spool]──(Sensor 2)

Notes:
- Cables converge at platform (center)
- Load cells measure cable tension
- Motors control cable length
- Sensors 0,1,2 track base positions
- Sensor 3 tracks platform position
```

---

## Testing Individual Components

### Test 1: Motors Only

**Goal**: Verify motors respond to commands

**Tools Needed**: Dynamixel Wizard 2.0

**Procedure**:
1. Connect motors (all 3 in daisy-chain)
2. Open Dynamixel Wizard
3. Scan and find all 3 motors (IDs: 1, 2, 3)
4. For each motor:
   - Enable torque
   - Set goal position (try 2048, 3000, 1000)
   - Motor should move
   - Read present position
5. Disable torque
6. Manually rotate motor - should spin freely

**Success Criteria**:
- All 3 motors found
- Motors respond to position commands
- No communication errors
- Motors hold position when torque enabled

### Test 2: Tracking Only

**Goal**: Verify trakSTAR provides accurate positions

**Tools Needed**: NDI diagnostic software

**Procedure**:
1. Power on driveBAY
2. Connect all 4 sensors
3. Run NDI diagnostic tool
4. Initialize system
5. Read each sensor:
   - Should show X, Y, Z coordinates
   - Quality > 0.8
6. Move sensors - positions should update
7. Measure known distance - verify accuracy

**Success Criteria**:
- All 4 sensors readable
- Positions update in real-time
- Quality indicators good
- Reasonable position values (mm)

### Test 3: Load Cells Only

**Goal**: Verify load cell readings

**Tools Needed**: NI MAX

**Procedure**:
1. Connect DAQ to computer
2. Wire load cells to AI0, AI1, AI2
3. Open NI MAX → Test Panels
4. Read analog inputs continuously
5. For each load cell:
   - Note baseline voltage (no load)
   - Apply gentle tension
   - Voltage should increase
   - Release - returns to baseline

**Success Criteria**:
- Stable voltage readings
- Voltages change with tension
- No excessive noise (<0.1V ripple)
- All 3 channels working

---

## System Integration

### Integration Steps

**1. Combine All Components**
```
- Keep motors powered off initially
- Connect all USB devices
- Verify all connections
```

**2. Initial Power-On**
```
1. Power on computer
2. Power on driveBAY
3. Connect DAQ
4. Power on motors (last)
5. Wait 5 seconds for initialization
```

**3. Run C++ System**
```
cd build/Release
./system.run

Expected console output:
- Heartprinter System Starting...
- Initializing motors...
- Initializing tracker...
- Initializing DAQ...
- Calibration starting...
```

**4. Monitor Calibration**
```
System will automatically:
1. Move to Base 0 (Left)
2. Detect arrival (motor position stable)
3. Record base position from Sensor 0
4. Repeat for Base 1 (Center) and Base 2 (Right)
5. Calculate centroid
6. Move to centroid
7. Ready for commands
```

**5. First Movement Test**
```
- Start Python GUI (see Python README)
- System should show "READY"
- Set a nearby target position
- Click "Execute"
- Platform should move smoothly
- Watch load cell voltages (should stay in range)
```

---

## Troubleshooting

### Motor Issues

**Problem**: Motors don't respond
- Check power supply (should be 12V, enough current)
- Verify U2D2 connection
- Check motor IDs (must be unique: 1, 2, 3)
- Test with Dynamixel Wizard

**Problem**: Motor overheats
- Reduce load (lighter platform)
- Check for mechanical binding
- Verify proper voltage (12V recommended)

**Problem**: Communication errors
- Check baud rate (57600 for all motors)
- Verify cable connections
- Reduce cable length (if very long)
- Check for loose connectors

### Tracking Issues

**Problem**: Sensors not detected
- Verify driveBAY powered on
- Check USB connection
- Ensure drivers installed
- Test with NDI diagnostic tool

**Problem**: Low quality readings
- Move sensors away from metal objects
- Reduce distance from driveBAY
- Check for EM interference sources
- Ensure sensors not damaged

**Problem**: Erratic positions
- Check cable connections
- Verify sensors are stationary (for bases)
- Reduce EM interference (power supplies, motors)
- Recalibrate if needed

### Load Cell Issues

**Problem**: No voltage reading
- Check wiring (signal, excitation, ground)
- Verify DAQ device recognized
- Test with NI MAX
- Check load cell power (if external)

**Problem**: Noisy readings
- Use differential mode
- Shorten cables
- Add low-pass filter (hardware or software)
- Move DAQ away from motors

**Problem**: Incorrect scaling
- Recalibrate load cells
- Verify voltage range settings
- Check for mechanical issues (friction)

### Integration Issues

**Problem**: System won't initialize
- Check all hardware connected
- Review console/log errors
- Test components individually
- Verify driver installations

**Problem**: Calibration fails
- Ensure base sensors (0,1,2) are stationary
- Check motor operation
- Verify tracking quality
- Increase calibration timeout (in code)

**Problem**: Unstable during operation
- Check load cell thresholds
- Verify cable routing (no snags)
- Adjust tension limits
- Check for mechanical binding

---

## Maintenance

### Regular Checks

**Daily** (before operation):
- Visual inspection of cables
- Check all connections
- Verify sensor placement
- Test emergency stop

**Weekly**:
- Clean sensors (dust can affect tracking)
- Check motor mounting screws
- Inspect load cell mounting
- Review log files for errors

**Monthly**:
- Recalibrate load cells
- Verify motor performance
- Update firmware if available
- Backup configuration

### Replacement Parts

**Common Wear Items**:
- Cables (fishing line, Kevlar thread)
- Load cells (mechanical fatigue)
- Motor horns (if damaged)
- Sensor cables

**Spare Parts to Keep**:
- Extra motor cable (JST connectors)
- Replacement fishing line
- Spare EM sensor
- Extra JST connectors

---

## Related Documentation

- [Main README](../README.md)
- [Architecture Documentation](ARCHITECTURE.md)
- [Build Guide](BUILD_GUIDE.md)
- [State Machine Documentation](STATE_MACHINE.md)
