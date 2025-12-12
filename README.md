# Heartprinter

<p align="center">
  <strong>A cable-driven parallel robot system for cardiac research applications</strong>
</p>

---

## Overview

Heartprinter is a precision 3-motor cable-driven parallel robot system designed for cardiac and medical research applications. The system provides real-time 3D position control and monitoring through electromagnetic tracking, enabling precise positioning for biomedical research and experimentation.

Developed at Carnegie Mellon University's Surgical Mechatronics Lab, Heartprinter integrates advanced motor control, position tracking, and load monitoring into a robust, cross-platform control system with an intuitive Python GUI interface.

---

## Key Features

- **Cable-Driven Parallel Robot**: 3-motor cable system for precise 3D positioning
- **Real-Time Position Tracking**: Northern Digital trakSTAR electromagnetic tracking
- **Load Monitoring**: 3-channel load cell monitoring for cable tension control
- **State Machine Architecture**: Robust control flow with automatic calibration
- **Safety Systems**: Real-time hardware safety checks and emergency stop
- **Cross-Platform**: Windows and Linux support
- **Python GUI**: User-friendly interface with real-time visualization
- **Data Analysis Tools**: Comprehensive logging and post-experiment analysis

---

## System Architecture

```
┌──────────────────────────────────────────────────────────┐
│                    Python GUI                            │
│           (Visualization & Control Interface)            │
└────────────────────┬─────────────────────────────────────┘
                     │ Shared Memory IPC
┌────────────────────▼─────────────────────────────────────┐
│              C++ Control System                          │
│            (Real-Time State Machine)                     │
└─────┬──────────────┬──────────────┬──────────────────────┘
      │              │              │
┌─────▼─────┐   ┌────▼─────┐   ┌────▼──────┐
│ 3x Motors │   │ trakSTAR │   │ Load Cells│
│(Dynamixel)│   │(Position)│   │(NI-DAQmx) │
└───────────┘   └──────────┘   └───────────┘
```

### Hardware Components

1. **Motion Control**: 3x Dynamixel XC430-T150BB-T servo motors
2. **Position Tracking**: Northern Digital trakSTAR electromagnetic tracking system (4 sensors)
3. **Load Monitoring**: 3x CALT DYZL-107 load cells with NI-DAQ USB-6421
4. **Computing Platform**: Windows 10/11 or Linux (Ubuntu/Debian)

---

## Quick Start

### Prerequisites

**Hardware:**
- 3x Dynamixel XC430-T150BB-T motors with U2D2 or USB2Dynamixel adapter
- Northern Digital trakSTAR with driveBAY interface
- NI-DAQ USB-6421 (or compatible)
- 3x load cells with amplifiers

**Software:**
- Windows 10/11 (64-bit) OR Linux (Ubuntu/Debian)
- CMake 3.10+
- Visual Studio 2019+ (Windows) OR GCC (Linux)
- Python 3.8+

### Installation

1. **Clone the repository:**
   ```bash
   git clone https://github.com/shreyassanghvi/Heartprinter.git
   cd Heartprinter
   ```

2. **Install hardware drivers:**
   - [Dynamixel SDK](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/overview/) (included in repo)
   - [Northern Digital trakSTAR API](https://www.ndigital.com/)
   - [NI-DAQmx](https://www.ni.com/en/support/downloads/drivers/download.ni-daq-mx.html)

3. **Build C++ control system:**
   ```bash
   cd code/Cpp
   mkdir build && cd build
   cmake .. -DCMAKE_BUILD_TYPE=Release
   cmake --build . --config Release
   ```

4. **Set up Python GUI:**
   ```bash
   cd code/Python
   pip install -r requirements.txt
   ```

5. **Run the system:**
   - Start Python GUI: `python gui/main.py`
   - Start C++ control system: `./build/system.run` (or `system.run.exe` on Windows)

---

## Documentation

Comprehensive documentation is organized as follows:

### C++ Control System
- **[C++ System README](code/Cpp/README.md)** - Main C++ documentation
- **[Architecture Guide](code/Cpp/docs/ARCHITECTURE.md)** - System design and components
- **[State Machine Documentation](code/Cpp/docs/STATE_MACHINE.md)** - Control flow and states
- **[Build Guide](code/Cpp/docs/BUILD_GUIDE.md)** - Detailed compilation instructions
- **[Hardware Setup](code/Cpp/docs/HARDWARE_SETUP.md)** - Physical setup and wiring

### Python GUI & Analysis
- **[Python System README](code/Python/README.md)** - GUI and analysis tools documentation
  - **[Unit Testing Guide](code/Python/test_gui/README.md)** - Comprehensive testing documentation (77 tests)
  - **[Test Results Report](code/Python/test_gui/FINAL_TEST_REPORT.md)** - Detailed test analysis and known bugs

### Project Management
- **[TAGS.md](TAGS.md)** - Git tagging strategy and version history

---

## Repository Structure

```
Heartprinter/
├── code/
│   ├── Cpp/                    # C++ real-time control system
│   │   ├── include/            # Header files
│   │   │   ├── custom/         # Heartprinter-specific headers
│   │   │   ├── Dynamixel_SDK/  # Motor control SDK
│   │   │   ├── Trackstar/      # Position tracking API
│   │   │   └── NIDAQmx/        # Load cell interface
│   │   ├── src/                # Source files
│   │   ├── docs/               # Detailed technical documentation
│   │   └── CMakeLists.txt      # Build configuration
│   └── Python/                 # Python GUI and analysis tools
│       ├── gui/                # User interface
│       └── tools/              # Data analysis scripts
├── spdlog/                     # Logging library (submodule)
├── TAGS.md                     # Version tagging documentation
└── README.md                   # This file
```

---

## Hardware Components & Specifications

### Motors
**Dynamixel XC430-T150BB-T** ([Datasheet](https://www.robotis.us/dynamixel-xc430-t150bb-t/))
- Stall Torque: 1.4 N⋅m (at 12V)
- No Load Speed: 46 RPM (at 12V)
- Operating Voltage: 9-12V
- Communication: TTL Half-Duplex UART
- Protocol: Dynamixel Protocol 2.0
- Resources:
  - [Dynamixel SDK](https://emanual.robotis.com/docs/en/dxl/dxl-quick-start-guide/#dynamixel-sdk)
  - [SDK Repository](https://github.com/ROBOTIS-GIT/DynamixelSDK/tree/main)
  - [Configuration Tool](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)

### Position Tracking
**Northern Digital trakSTAR**
- 3D electromagnetic tracking system
- 4 sensors (3 base positions + 1 moving platform)
- driveBAY interface
- Sub-millimeter accuracy
- Installation: [Setup Package](https://github.com/shreyassanghvi/Heartprinter/blob/main/code/10006809-%203DG%20driveBAY-trakSTAR%20installation%20package%20R03.zip)

### Data Acquisition
**NI-DAQ USB-6421** ([Specifications](https://www.ni.com/en-us/support/model.usb-6421.html))
- 3 analog input channels for load cell monitoring
- USB connectivity
- [Pinout Reference](https://www.ni.com/docs/en-US/bundle/usb-6421/page/pinout.html)
- [DAQmx SDK](https://www.ni.com/en/support/downloads/drivers/download.ni-daq-mx.html)
- [Getting Started Guide](https://www.ni.com/en/support/documentation/supplemental/06/getting-started-with-ni-daqmx--main-page.html)

### Load Sensors
**CALT DYZL-107 Tension Sensor** - 2 KG capacity ([Product Link](https://www.amazon.com/DYZL-107-Tension-Rollers-Measuring-Detecting/dp/B0BKL165PR?sr=8-2))

**JY-S85 Load Cell Amplifier** ([Product Link](https://www.amazon.com/FEGIANCHE-Current-Amplifier-Transmitter-Voltage/dp/B0DBD3DTYW?th=1))

---

## Project Status

**Current Version:** v0.9.0 (Release Candidate)

**Status:** Feature complete and ready for testing

**Working Features:**
- ✅ Complete state machine implementation
- ✅ 3-motor synchronized control
- ✅ Real-time position tracking
- ✅ Load monitoring and tension adjustment
- ✅ Automatic calibration
- ✅ Python GUI with visualization
- ✅ Data logging and analysis
- ✅ Cross-platform build system

**Known Limitations:**
- ⚠️ macOS not supported (hardware driver limitations)
- ⚠️ Linux NI-DAQmx compatibility requires validation
- ⚠️ System requires hardware supervision during operation

**Upcoming (v1.0.0):**
- Enhanced documentation
- Ubuntu/Linux NI-DAQmx testing and validation
- Performance optimization
- Production-ready configuration

See [TAGS.md](TAGS.md) for complete version history and roadmap.

---

## Safety & Best Practices

⚠️ **IMPORTANT SAFETY INFORMATION**

This system controls physical hardware that can cause injury or equipment damage if not operated properly.

**Before Operation:**
- ✓ Verify all hardware connections are secure
- ✓ Ensure workspace is clear of obstructions
- ✓ Test emergency stop functionality
- ✓ Confirm load cells are properly calibrated

**During Operation:**
- ✓ Never leave system running unattended
- ✓ Monitor cable tension continuously
- ✓ Keep hands clear of moving parts
- ✓ Use emergency stop (Ctrl+C or GUI "End" button) if needed

**Emergency Stop:**
- Press `Ctrl+C` in the C++ control terminal
- Click "End" button in Python GUI
- Power cycle motors if system becomes unresponsive

---

## Contributing

This project is part of ongoing research at Carnegie Mellon University's Surgical Mechatronics Lab. For questions or collaboration inquiries, please contact the project team or open an issue.

**Development Guidelines:**
- Test all changes with actual hardware when possible
- Update documentation for any API or behavior changes
- Follow existing code style and patterns
- Add logging for significant events
- Never bypass safety checks

---

## Acknowledgments

### Development Team

**Lead Developer & System Integration:**
- Shreyas Sanghvi, Carnegie Mellon University

**Software Development & Code Optimization:**
- Juan Muerto, Carnegie Mellon University
- Shreyas Sanghvi, Carnegie Mellon University

**Mechanical Design & CAD:**
- Andrea Piraneque, Carnegie Mellon University

**System Testing & Validation:**
- Juan Muerto, Carnegie Mellon University
- Maggie Wu, Carnegie Mellon University
- Shreyas Sanghvi, Carnegie Mellon University


### Advisor

**Principal Investigator:**
- Dr. Cameron Riviere, Carnegie Mellon University

### Institution

This project was developed at the **Surgical Mechatronics Laboratory** in the Robotics Institute at **Carnegie Mellon University**.

### Funding

This work was supported by Carnegie Mellon University Scholarly Funding (formerly GuSH).

---

## License

This project is licensed under the Apache License 2.0 - see the [LICENSE](LICENSE) file for details.

```
Copyright 2025 Surgical Mechatronics Laboratory, Carnegie Mellon University

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
```

---

## References

### Related Publications

The HeartPrinter system and its design principles are described in the following peer-reviewed publications:

1. **Ladak, A., Hajjar, R. J., Murali, S., Michalek, J. J., & Riviere, C. N.** (2023). "Cable Tension Optimization for an Epicardial Parallel Wire Robot." *Journal of Medical Devices*, *17*(2), 021006. [https://doi.org/10.1115/1.4056866](https://doi.org/10.1115/1.4056866)

2. **Ladak, A., Dixit, D., Halbreiner, M. S., Passineau, M. J., Murali, S., & Riviere, C. N.** (2021). "Introducer Design Concepts for an Epicardial Parallel Wire Robot." *Robotic Surgery: Research and Reviews*, *8*, 21-38. [https://doi.org/10.2147/RSRR.S327069](https://doi.org/10.2147/RSRR.S327069)

### Technical Documentation

The following technical manuals and datasheets are included in the `literature/` folder:

1. **Northern Digital Inc.** "3D Guidance API Guide R01." Document DOC-10006822. [literature/DOC-10006822- 3D Guidance API Guide R01.pdf](literature/DOC-10006822-%203D%20Guidance%20API%20Guide%20R01.pdf)

2. **Northern Digital Inc.** "trakSTAR MUS User Guide R01." Document DOC-10006825. [literature/DOC-10006825- trakSTAR MUS User Guide R01.pdf](literature/DOC-10006825-%20trakSTAR%20MUS%20User%20Guide%20R01.pdf)

### Background Research Literature

3. **Qian, S., Zi, B., Shang, W.-W., & Xu, Q.** "A Review of Cable-Driven Parallel Robots: Typical Configurations, Analysis Techniques, and Control Methods." [literature/A_Review_of_Cable-Driven_Parallel_Robots_Typical_Configurations_Analysis_Techniques_and_Control_Methods.pdf](literature/A_Review_of_Cable-Driven_Parallel_Robots_Typical_Configurations_Analysis_Techniques_and_Control_Methods.pdf)

4. **Jin, X., Zhang, H., Wang, L., & Li, Q.** (2024). "Review on Control Strategies for Cable-Driven Parallel Robots with Model Uncertainties." *Chinese Journal of Mechanical Engineering*, 37(156). DOI: 10.1186/s10033-024-01149-8. [literature/s10033-024-01149-8.pdf](literature/s10033-024-01149-8.pdf)

---

## Contact

For questions, issues, or collaboration inquiries:
- **Repository Issues:** [GitHub Issues](https://github.com/shreyassanghvi/Heartprinter/issues)
- **Lab Website:** [Surgical Mechatronics Lab](https://www.ri.cmu.edu/)

---

**Carnegie Mellon University | Robotics Institute | Surgical Mechatronics Laboratory**
