# Heartprinter C++ System

The C++ codebase is the core control system for the Heartprinter hardware. It manages real-time motor control, position tracking, load cell monitoring, and coordinates all hardware components through a state machine architecture.

---

## ⚠️ IMPORTANT: Platform Compatibility

**macOS IS NOT SUPPORTED**

This system requires NIDAQmx and trakSTAR hardware drivers which are only available for Windows and Linux.

**Supported Platforms:**
- ✅ Windows 10/11 (64-bit only)
- ✅ Linux (Ubuntu, Debian, etc.)
- ❌ macOS (Not Compatible)

---

## Table of Contents

1. [System Overview](#system-overview)
2. [Project Structure](#project-structure)
3. [Hardware Requirements](#hardware-requirements)
4. [Software Dependencies](#software-dependencies)
5. [Building the Project](#building-the-project)
6. [Running the System](#running-the-system)
7. [Configuration](#configuration)
8. [Architecture Documentation](#architecture-documentation)
9. [Troubleshooting](#troubleshooting)

---

## System Overview

The Heartprinter C++ system is a real-time control system that:

- **Controls 3 Dynamixel motors** for cable-driven parallel positioning
- **Tracks 3D position** using Northern Digital trakSTAR electromagnetic tracking system
- **Monitors cable tension** via 3-channel load cells connected to NI-DAQmx
- **Communicates with Python GUI** through Windows shared memory
- **Implements safety checks** to prevent hardware damage
- **Logs all operations** for analysis and debugging

### Key Features

- **State Machine Architecture**: Robust state-based control flow
- **Real-time Control Loop**: Sub-millisecond motor response times
- **Automatic Calibration**: Self-calibration routine for base position detection
- **Safety System**: Hardware safety checks prevent dangerous operations
- **Shared Memory IPC**: Efficient communication with Python GUI
- **Comprehensive Logging**: Detailed logs with spdlog library

---

## Project Structure

```
code/Cpp/
├── README.md                          # This file
├── docs/                              # Documentation (sub-READMEs)
│   ├── ARCHITECTURE.md                # System architecture
│   ├── BUILD_GUIDE.md                 # Detailed build instructions
│   ├── HARDWARE_SETUP.md              # Hardware setup guide
│   └── STATE_MACHINE.md               # State machine documentation
├── CMakeLists.txt                     # CMake build configuration
├── include/                           # Header files
│   ├── custom/                        # Heartprinter-specific headers
│   │   ├── SystemController.h         # Main controller
│   │   ├── Motor.h                    # Motor control
│   │   ├── CTrackstar.h               # Position tracking
│   │   ├── NIDAQ.h                    # Load cell interface
│   │   ├── SharedMemoryManager.h      # IPC with Python
│   │   └── Init.h                     # Initialization utilities
│   ├── Dynamixel_SDK/                 # Dynamixel motor SDK
│   ├── Trackstar/                     # trakSTAR tracking API
│   ├── NIDAQmx/                       # NI-DAQmx headers
│   └── spdlog/                        # Logging library
├── src/                               # Source files
│   ├── custom/                        # Heartprinter-specific source
│   │   ├── SystemController.cpp       # Main control logic
│   │   ├── Motor.cpp                  # Motor implementations
│   │   ├── CTrackstar.cpp             # Tracking implementations
│   │   ├── NIDAQ.cpp                  # DAQ implementations
│   │   ├── SharedMemoryManager.cpp    # Shared memory handling
│   │   └── Init.cpp                   # System initialization
│   └── dynamixel_sdk/                 # Dynamixel SDK source
└── logs/                              # Runtime logs (generated)
```

---

## Hardware Requirements

### Required Hardware

1. **3x Dynamixel Motors**
   - Model: [XC430-T150BB-T](https://www.robotis.us/dynamixel-xc430-t150bb-t/)
   - Interface: TTL (3-wire) via U2D2 or USB2Dynamixel
   - Communication: Half-Duplex UART (0-5V TTL logic)
   - Baud Rate: 57600
   - Protocol: Dynamixel Protocol 2.0

2. **Northern Digital trakSTAR**
   - 3D electromagnetic tracking system
   - Minimum 4 sensors (3 bases + 1 moving platform)
   - driveBAY interface

3. **National Instruments DAQ Device**
   - 3-channel analog input capability
   - Compatible with NI-DAQmx driver
   - Used for load cell monitoring

4. **3x Load Cells**
   - Analog output (voltage)
   - Connected to NI-DAQ analog inputs
   - For cable tension monitoring

5. **Computer**
   - Windows 10/11 64-bit OR Linux
   - Minimum: 4GB RAM, quad-core processor
   - USB ports for motor controller
   - Available for trakSTAR driveBAY connection

### Hardware Connections

```
┌──────────────┐
│   Computer   │
└──────┬───────┘
       │
       ├─[USB]──────> U2D2/USB2Dynamixel ──[TTL 3-wire]──> 3x Dynamixel Motors
       │
       ├─[driveBAY]─> trakSTAR System ─────────────────> 4x EM Sensors
       │
       └─[USB/PCI]──> NI-DAQ Device ───[Analog]───────> 3x Load Cells
```

---

## Software Dependencies

### Required Software

#### 1. **3D Guidance driveBAY trakSTAR API**
   - Version: R03 or later
   - Download: Available from Northern Digital Inc.
   - Installation Path (Windows): `C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\`
   - **Required Files:**
     - `ATC3DG64.dll` / `ATC3DG64.lib`
     - `ATC3DG.h`

#### 2. **NI-DAQmx**
   - Version: 20.x or later
   - Download: [NI-DAQmx Download](https://www.ni.com/en/support/downloads/drivers/download.ni-daq-mx.html)
   - Installation Path (Windows): `C:\Program Files (x86)\National Instruments\`
   - **Required Files:**
     - `NIDAQmx.lib`
     - `NIDAQmx.h`

#### 3. **Dynamixel SDK**
   - Version: 3.7.x or later
   - **Included in this repository** (src/dynamixel_sdk/)
   - Documentation: [Dynamixel SDK Manual](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/)

#### 4. **spdlog**
   - Version: 1.x
   - **Included as submodule** (../../spdlog/)
   - Fast C++ logging library

### Build Tools

#### Windows
- **CMake** 3.10 or higher
- **Visual Studio 2019 or later** (with C++ tools)
  - Or **MinGW-w64** (64-bit GCC for Windows)
- **Git** (for cloning repository)

#### Linux
```bash
sudo apt-get update
sudo apt-get install build-essential cmake git
```

---

## Building the Project

### Quick Build (Windows)

```bash
# Navigate to the C++ directory
cd code/Cpp

# Create build directory
mkdir build
cd build

# Generate build files (Visual Studio)
cmake .. -G "Visual Studio 16 2019" -A x64

# Build
cmake --build . --config Release

# Output: build/Release/system.run.exe
```

### Quick Build (Linux)

```bash
# Navigate to the C++ directory
cd code/Cpp

# Create build directory
mkdir build
cd build

# Generate build files
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build
make -j$(nproc)

# Output: build/system.run
```

### Detailed Build Instructions

For comprehensive build instructions, troubleshooting, and alternative build methods, see:
**[📖 docs/BUILD_GUIDE.md](docs/BUILD_GUIDE.md)**

---

## Running the System

### Prerequisites

1. **Hardware Setup Complete**
   - All hardware connected and powered
   - See [docs/HARDWARE_SETUP.md](docs/HARDWARE_SETUP.md) for details

2. **Python GUI Running** (optional but recommended)
   - See `../Python/README.md` for instructions
   - Provides visual interface and control

### Running the Executable

**Windows:**
```bash
cd build/Release
.\system.run.exe
```

**Linux:**
```bash
cd build
./system.run
```

### What Happens on Startup

1. **Initialization Phase**
   - Connects to Dynamixel motors
   - Initializes trakSTAR tracking system
   - Starts NI-DAQ load cell monitoring
   - Creates shared memory for Python communication

2. **Calibration Phase**
   - Automatically moves to detect 3 base positions
   - Calculates workspace centroid
   - Validates sensor placement

3. **Ready State**
   - System waits for commands from Python GUI
   - Monitors all sensors
   - Logs status updates

4. **Operation**
   - Accepts target positions from GUI
   - Moves to targets with safety checks
   - Adjusts cable tension automatically
   - Logs all movements and errors

### Command Line Options

Currently, the system does not accept command-line arguments. Configuration is done via:
- Hardcoded constants in `SystemController.h`
- Shared memory commands from Python GUI

---

## Configuration

### System Configuration

Key parameters can be adjusted in `include/custom/SystemController.h`:

```cpp
struct SystemConfig {
    // Motor configuration
    std::string deviceName = "/dev/ttyUSB0";  // Linux: /dev/ttyUSB0, Windows: COM3
    int baudRate = 57600;

    // Safety limits
    double maxLoadVoltage = -0.5;      // Max tension (volts)
    double minLoadVoltage = -1.0;      // Min tension (volts)

    // Control parameters
    int tensionAdjustmentSteps = 2;    // Motor steps for tension correction
    double posErrorThreshold = 1.5;    // Position tolerance (mm)
    double angleThreshold = 7.0;       // Angle tolerance (degrees)

    // Calibration
    int calibrationStabilityReads = 10;
    double calibrationPositionTolerance = 5.0;
};
```

**Note:** After changing configuration, rebuild the project.

### Motor Configuration

**Motor Model**: Dynamixel XC430-T150BB-T
- **Control Mode**: Extended Position Control Mode
- **Resolution**: 4096 units per revolution
- **Stall Torque**: 1.4 N⋅m (at 12V)
- **No Load Speed**: 46 RPM (at 12V)
- **Operating Voltage**: 9-12V (Recommended: 12V)
- **Communication**: TTL (3-wire, Half Duplex UART, 0-5V logic)

**Motor IDs**:
- **Motor 0** (Base Left): ID = 1
- **Motor 1** (Base Center): ID = 2
- **Motor 2** (Base Right): ID = 3

IDs can be changed using [Dynamixel Wizard 2.0](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/).

### Log Files

Logs are automatically created in `logs/` directory:
- Filename format: `log_YYYYMMDD_HHMMSS.txt`
- Contains: Timestamps, state transitions, position data, errors
- Analyzed by Python tools (see `../Python/tools/log_analysis.py`)

---

## Architecture Documentation

### System Architecture

The system uses a **state machine architecture** for robust control:

```
┌──────────────────────────────────────────────────────────────┐
│                     SystemController                          │
│                    (State Machine Core)                       │
└───────────┬──────────────────────────────────────┬───────────┘
            │                                      │
    ┌───────▼────────┐                    ┌───────▼────────┐
    │  Motor Control │                    │   Tracking     │
    │  (Dynamixel)   │                    │  (trakSTAR)    │
    └────────────────┘                    └────────────────┘
            │                                      │
    ┌───────▼────────┐                    ┌───────▼────────┐
    │  Load Cells    │                    │ Shared Memory  │
    │  (NI-DAQmx)    │                    │    (Python)    │
    └────────────────┘                    └────────────────┘
```

### State Machine

The system operates through these states:

1. **START** → Initial state
2. **INITIALIZING** → Hardware initialization
3. **CALIBRATION** → Automatic base detection
4. **READY** → Waiting for commands
5. **RUNNING** → Position reached, monitoring
6. **MOVING** → Actively moving to target
7. **TENSION** → Adjusting cable tension
8. **ERROR** → Error state
9. **END** → Shutdown requested
10. **CLEANUP** → Cleaning up resources

For detailed state machine documentation:
**[📖 docs/STATE_MACHINE.md](docs/STATE_MACHINE.md)**

### Component Documentation

- **[Architecture Overview](docs/ARCHITECTURE.md)** - System design and component interactions
- **[Hardware Setup Guide](docs/HARDWARE_SETUP.md)** - Physical setup and wiring
- **[Build Guide](docs/BUILD_GUIDE.md)** - Detailed compilation instructions

---

## Troubleshooting

### Common Issues

#### Motors Don't Connect
- **Check USB connection** to U2D2/USB2Dynamixel
- **Verify port name**: Windows (`COM3`), Linux (`/dev/ttyUSB0`)
- **Check motor IDs**: Use Dynamixel Wizard to scan
- **Verify baud rate**: Should be 57600

#### trakSTAR Not Found
- **Install driveBAY API**: Must be installed before building
- **Check driveBAY connection**: Ensure driveBAY is powered and connected
- **Run trakSTAR diagnostic tools**: Test sensors independently

#### NI-DAQ Errors
- **Install NI-DAQmx driver**: Required even if no DAQ hardware
- **Check device name** in NI MAX (National Instruments Measurement & Automation Explorer)
- **Verify analog input channels**: CH0, CH1, CH2

#### Build Errors
- **Missing dependencies**: Ensure all SDKs are installed
- **Wrong architecture**: Must use 64-bit build (x64)
- **Path issues**: Check CMakeLists.txt paths match your installation

#### Shared Memory Issues
- **Python GUI not running**: Start Python GUI first
- **Access denied**: Run with administrator privileges on Windows
- **Stale memory**: Restart both C++ and Python applications

### Getting Help

1. **Check logs**: Look in `logs/` directory for detailed error messages
2. **Verify hardware**: Test each component independently
3. **Review documentation**: See sub-README files in `docs/`
4. **Check Python logs**: Python GUI also creates logs during operation

---

## Safety Notes

⚠️ **IMPORTANT SAFETY INFORMATION**

- **Emergency Stop**: Press Ctrl+C or use GUI "End" button
- **Workspace Limits**: System validates positions before movement
- **Tension Monitoring**: Load cells prevent cable over-tension
- **Always supervise**: Never leave system running unattended
- **Power cycle motors**: If motors behave erratically, power cycle them

---

## Development

### Contributing

When modifying the C++ code:

1. **Test thoroughly**: Hardware control code must be reliable
2. **Update documentation**: Keep README and sub-docs in sync
3. **Add logging**: Use spdlog for all significant events
4. **Follow style**: Maintain existing code style and patterns
5. **Safety first**: Never bypass safety checks

### Code Style

- **Indentation**: 4 spaces
- **Naming**:
  - Classes: `PascalCase`
  - Functions: `camelCase`
  - Variables: `camelCase`
  - Constants: `UPPER_CASE`
- **Comments**: Use `//` for single-line, `/* */` for blocks
- **Error handling**: Use exceptions (`SystemException`)

---

## License

[Your license information here]

---

## Related Documentation

- **[Python Code README](../Python/README.md)** - GUI and analysis tools
- **[Project Root README](../../README.md)** - Overall project documentation
- **[Architecture Details](docs/ARCHITECTURE.md)** - System design
- **[Build Guide](docs/BUILD_GUIDE.md)** - Detailed build instructions
- **[Hardware Setup](docs/HARDWARE_SETUP.md)** - Physical setup guide
- **[State Machine](docs/STATE_MACHINE.md)** - Control flow documentation
