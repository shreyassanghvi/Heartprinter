# Heartprinter C++ Build Guide

Comprehensive guide for building the Heartprinter C++ system from source.

---

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Platform-Specific Setup](#platform-specific-setup)
3. [Installing Dependencies](#installing-dependencies)
4. [Building with CMake](#building-with-cmake)
5. [Build Configurations](#build-configurations)
6. [Troubleshooting Build Issues](#troubleshooting-build-issues)
7. [Verifying the Build](#verifying-the-build)

---

## Prerequisites

### Required Software

#### All Platforms
- **Git**: For cloning the repository
- **CMake**: Version 3.10 or higher
- **C++ Compiler**: Supporting C++17

#### Windows
- **Visual Studio 2019 or later** (Community Edition is fine)
  - Or **MinGW-w64** (64-bit)
- **CMake GUI** (optional, but helpful)

#### Linux
- **GCC** 7.0 or later
- **Build tools**: make, etc.

### Hardware Requirements for Building

**Note**: You can build the code WITHOUT having the hardware connected, but you must install the software drivers/SDKs first.

---

## Platform-Specific Setup

### Windows Setup

#### Option 1: Visual Studio (Recommended)

**1. Install Visual Studio 2019/2022**
```
Download from: https://visualstudio.microsoft.com/downloads/
During installation, select:
  - Desktop development with C++
  - Windows 10/11 SDK
  - CMake tools for Visual Studio (optional)
```

**2. Install CMake**
```
Download from: https://cmake.org/download/
During installation:
  - Check "Add CMake to system PATH"
```

**3. Verify Installation**
```cmd
cmake --version
# Should show: cmake version 3.10.0 or higher
```

#### Option 2: MinGW-w64

**1. Install MinGW-w64**
```
Download from: https://www.mingw-w64.org/downloads/
Recommended: Use MSYS2 installer
```

**2. Install via MSYS2**
```bash
# Open MSYS2 terminal
pacman -S mingw-w64-x86_64-gcc
pacman -S mingw-w64-x86_64-cmake
pacman -S make
```

**3. Add to PATH**
```
Add to system PATH:
  C:\msys64\mingw64\bin
```

### Linux Setup

**Ubuntu/Debian**
```bash
# Update package list
sudo apt-get update

# Install build essentials
sudo apt-get install build-essential cmake git

# Verify installation
gcc --version    # Should be 7.0 or higher
cmake --version  # Should be 3.10 or higher
```

**Fedora/RHEL**
```bash
sudo dnf install gcc gcc-c++ cmake git make
```

**Arch Linux**
```bash
sudo pacman -S base-devel cmake git
```

---

## Installing Dependencies

### 1. Northern Digital trakSTAR API

**Windows**

```
1. Download: 3DG driveBAY trakSTAR installation package R03
   Contact: Northern Digital Inc. or check their website

2. Run installer: 10006809-3DG driveBAY-trakSTAR installation package R03.exe

3. Default installation path:
   C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\

4. Verify files exist:
   - C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API\ATC3DG64.dll
   - C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API\ATC3DG64.lib
   - C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API\ATC3DG.h
```

**Linux**

```bash
# trakSTAR drivers for Linux
# Contact Northern Digital Inc. for Linux SDK
# Or check: https://www.ndigital.com/

# Install to: /usr/local/lib/trakstar/
# Update CMakeLists.txt paths accordingly
```

### 2. NI-DAQmx

**Windows**

```
1. Download NI-DAQmx driver:
   https://www.ni.com/en/support/downloads/drivers/download.ni-daq-mx.html

2. Run installer (requires ~4GB disk space)

3. Installation will create:
   C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc\NIDAQmx.lib
   C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\include\NIDAQmx.h

4. Optional: Install NI MAX (Measurement & Automation Explorer)
   - Useful for testing DAQ devices
   - Included with NI-DAQmx installer
```

**Linux**

```bash
# Download NI-DAQmx for Linux
# https://www.ni.com/en-us/support/downloads/drivers/download.ni-daqmx.html

# Follow installation instructions for your distribution
# Typically installs to /usr/local/natinst/

# Verify installation
ni-daqmx-config --version
```

### 3. Dynamixel SDK

**Already Included**

The Dynamixel SDK is included in this repository under `src/dynamixel_sdk/` and `include/Dynamixel_SDK/`.

No additional installation needed.

### 4. spdlog (Logging Library)

**Already Included as Submodule**

```bash
# If you cloned without submodules, initialize them:
cd /path/to/Heartprinter
git submodule update --init --recursive

# This will download spdlog into the spdlog/ directory
```

**Verify spdlog exists:**
```bash
ls ../../spdlog/include/spdlog/
# Should show: spdlog.h and other headers
```

---

## Building with CMake

### Method 1: Command Line (All Platforms)

**Windows (Visual Studio)**

```cmd
REM Navigate to C++ directory
cd C:\path\to\Heartprinter\code\Cpp

REM Create build directory
mkdir build
cd build

REM Generate Visual Studio project files
cmake .. -G "Visual Studio 16 2019" -A x64

REM Build (Release configuration)
cmake --build . --config Release

REM Or build Debug
cmake --build . --config Debug

REM Output will be in:
REM build/Release/system.run.exe
REM or
REM build/Debug/system.run.exe
```

**Windows (MinGW)**

```cmd
cd C:\path\to\Heartprinter\code\Cpp

mkdir build
cd build

REM Use MinGW Makefiles generator
cmake .. -G "MinGW Makefiles" -DCMAKE_BUILD_TYPE=Release

REM Build
mingw32-make -j4

REM Output: build/system.run.exe
```

**Linux**

```bash
# Navigate to C++ directory
cd /path/to/Heartprinter/code/Cpp

# Create build directory
mkdir build
cd build

# Generate Makefiles
cmake .. -DCMAKE_BUILD_TYPE=Release

# Build (use all cores)
make -j$(nproc)

# Output: build/system.run
```

### Method 2: CMake GUI (Windows)

**1. Open CMake GUI**
```
Start → CMake (cmake-gui)
```

**2. Set Paths**
```
Where is the source code: C:/path/to/Heartprinter/code/Cpp
Where to build the binaries: C:/path/to/Heartprinter/code/Cpp/build
```

**3. Configure**
```
Click "Configure"
Select generator: Visual Studio 16 2019
Select platform: x64
Click "Finish"
```

**4. Verify Paths**
```
Check that CMake found:
  - ATC3DG64.lib
  - NIDAQmx.lib
  - spdlog headers

If paths are wrong, manually edit them in the CMake GUI
```

**5. Generate**
```
Click "Generate"
```

**6. Build**
```
Click "Open Project" to open in Visual Studio
Or build from command line:
  cmake --build build --config Release
```

### Method 3: Visual Studio Integrated

**Visual Studio 2019/2022 with CMake Integration**

```
1. Open Visual Studio
2. File → Open → Folder
3. Select: C:\path\to\Heartprinter\code\Cpp
4. Visual Studio will automatically detect CMakeLists.txt
5. Wait for CMake configuration to complete
6. Select build configuration: x64-Release or x64-Debug
7. Build → Build All
```

---

## Build Configurations

### Release vs Debug

**Release** (Recommended for normal use)
```bash
# Optimized code, no debug symbols
# Faster execution
# Smaller binary size

# Command line:
cmake .. -DCMAKE_BUILD_TYPE=Release
```

**Debug** (For development)
```bash
# No optimization, includes debug symbols
# Slower execution
# Easier debugging with GDB/Visual Studio

# Command line:
cmake .. -DCMAKE_BUILD_TYPE=Debug
```

### Custom Build Options

**Change Compiler Flags**

Edit `CMakeLists.txt`:
```cmake
# Line 26: Compiler options
add_compile_options(-O3 -fPIC -pthread -Wall -m64 -g)

# Change -O3 to -O2 for less aggressive optimization
# Remove -g to exclude debug symbols
# Add -Werror to treat warnings as errors
```

**Custom Install Prefix**
```bash
cmake .. -DCMAKE_INSTALL_PREFIX=/custom/install/path
make install
```

---

## Troubleshooting Build Issues

### Common Issues and Solutions

#### 1. CMake Can't Find Dependencies

**Problem**: CMake error about missing ATC3DG64.lib or NIDAQmx.lib

**Solution**:
```cmake
# Edit CMakeLists.txt and verify paths match your installation:

# Line 30: trakSTAR include
"C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/"

# Line 69: trakSTAR library
"C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/ATC3DG64.lib"

# Line 76: NI-DAQmx library
"C:/Program Files (x86)/National Instruments/Shared/ExternalCompilerSupport/C/lib64/msvc/NIDAQmx.lib"
```

**Alternative**: Set environment variables
```cmd
REM Windows
set TRAKSTAR_DIR=C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API
set NIDAQMX_DIR=C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C

REM Then run CMake
cmake ..
```

#### 2. spdlog Not Found

**Problem**: CMake error about spdlog

**Solution**:
```bash
# Initialize git submodules
cd /path/to/Heartprinter
git submodule update --init --recursive

# Verify spdlog exists
ls spdlog/include/spdlog/spdlog.h
```

#### 3. Wrong Architecture (32-bit vs 64-bit)

**Problem**: Error about mismatched architecture

**Solution**:
```bash
# Windows: Force 64-bit
cmake .. -A x64

# Or use 64-bit generator
cmake .. -G "Visual Studio 16 2019" -A x64
```

**Verify**:
```cmake
# CMakeLists.txt line 12-14 enforces 64-bit
if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
    message(FATAL_ERROR "Only 64-bit builds are supported")
endif()
```

#### 4. Linker Errors

**Problem**: Unresolved external symbols during linking

**Possible Causes**:
1. Missing library files (.lib or .dll)
2. Wrong library architecture (32-bit vs 64-bit)
3. Incompatible compiler

**Solution**:
```bash
# Verify all .lib files exist
# Windows:
dir "C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API\*.lib"
dir "C:\Program Files (x86)\National Instruments\Shared\ExternalCompilerSupport\C\lib64\msvc\*.lib"

# Check they are 64-bit
# Use dumpbin (Visual Studio tool):
dumpbin /headers NIDAQmx.lib | findstr machine
# Should show: machine (x64)
```

#### 5. Missing DLLs at Runtime

**Problem**: Program builds but won't run - DLL not found

**Solution**:
```cmd
REM The build process should automatically copy DLLs
REM If not, manually copy:

REM From:
C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API\ATC3DG64.dll

REM To:
C:\path\to\Heartprinter\code\Cpp\build\Release\ATC3DG64.dll

REM Or add to system PATH:
set PATH=%PATH%;C:\Program Files (x86)\Northern Digital Inc\3D Guidance driveBAY\3DG API Developer\3DG API
```

**Automated Copy** (already in CMakeLists.txt):
```cmake
# Lines 109-121: Post-build DLL copy commands
add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
    "C:/Program Files (x86)/Northern Digital Inc/.../ATC3DG64.dll"
    $<TARGET_FILE_DIR:${PROJECT_NAME}>
)
```

#### 6. C++17 Compilation Errors

**Problem**: Compiler doesn't support C++17 features

**Solution**:
```bash
# Update compiler:
# GCC: 7.0 or higher
# MSVC: Visual Studio 2017 or higher
# Clang: 5.0 or higher

# Check version:
gcc --version
cl.exe  # (MSVC, run from VS developer command prompt)
```

#### 7. Permission Errors (Windows)

**Problem**: Access denied when building

**Solution**:
```cmd
REM Run as Administrator:
REM Right-click Visual Studio → Run as Administrator

REM Or run command prompt as admin:
REM Right-click cmd.exe → Run as Administrator
cd path\to\project
cmake --build build --config Release
```

#### 8. Out of Memory During Build

**Problem**: Compiler runs out of memory

**Solution**:
```bash
# Reduce parallel builds
# Instead of: make -j$(nproc)
make -j2  # Use only 2 parallel jobs

# Or on Windows:
cmake --build . --config Release -j 2
```

---

## Verifying the Build

### 1. Check Output File

**Windows**:
```cmd
REM Release build:
dir build\Release\system.run.exe

REM Debug build:
dir build\Debug\system.run.exe

REM Should see file ~500KB - 2MB
```

**Linux**:
```bash
ls -lh build/system.run
# Should see file ~500KB - 2MB
```

### 2. Check Dependencies (Windows)

```cmd
REM Use Dependency Walker or dumpbin
cd build\Release

REM List DLL dependencies:
dumpbin /dependents system.run.exe

REM Should show:
REM - ATC3DG64.dll
REM - KERNEL32.dll
REM - MSVCR*.dll (Visual C++ runtime)
REM - and other Windows system DLLs
```

### 3. Quick Run Test (Without Hardware)

```cmd
REM The program will fail if hardware isn't connected
REM But should start and give meaningful error messages

cd build\Release
.\system.run.exe

REM Expected output (without hardware):
REM [timestamp] [I]: Heartprinter System Starting...
REM [timestamp] [E]: Failed to initialize motors: Port not found
REM ... or similar error messages
```

### 4. Static Analysis (Optional)

```bash
# Linux: Check for common issues
cppcheck --enable=all --std=c++17 src/

# Output should have minimal warnings
```

---

## Advanced Build Options

### Building with Different Generators

**Ninja** (Faster builds):
```bash
# Install Ninja
# Windows: choco install ninja
# Linux: sudo apt-get install ninja-build

cmake .. -G Ninja -DCMAKE_BUILD_TYPE=Release
ninja
```

**Makefiles** (Windows with MinGW):
```cmd
cmake .. -G "MinGW Makefiles"
mingw32-make
```

### Cross-Compilation

**Building for Linux on Windows (WSL)**:
```bash
# In WSL terminal
cd /mnt/c/path/to/Heartprinter/code/Cpp

mkdir build-linux
cd build-linux

cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)
```

### Installing Build Artifacts

```bash
# Install to system (requires sudo on Linux)
cmake --install build --prefix /usr/local

# Or to custom directory:
cmake --install build --prefix ~/heartprinter-install

# Installed files:
# - bin/system.run (executable)
# - lib/libdynamixel_sdk.so (shared library)
# - include/*.h (headers)
```

---

## Build Performance Tips

### Speeding Up Builds

**1. Use Parallel Builds**
```bash
# Linux
make -j$(nproc)  # Use all cores

# Windows
cmake --build . --config Release -j %NUMBER_OF_PROCESSORS%
```

**2. Use ccache (Linux)**
```bash
# Install
sudo apt-get install ccache

# Configure
export CC="ccache gcc"
export CXX="ccache g++"

cmake ..
make
```

**3. Incremental Builds**
```bash
# Only rebuild changed files
make

# Clean rebuild (if needed)
make clean
make
```

**4. Use Ninja Instead of Make**
```bash
cmake .. -G Ninja
ninja  # Usually faster than make
```

---

## Clean Builds

### When to Clean

- After changing CMakeLists.txt
- After changing compiler flags
- When experiencing strange build errors
- When switching between Debug/Release

### How to Clean

**Complete Clean** (removes everything):
```bash
# Delete entire build directory
rm -rf build/  # Linux
rmdir /s build  # Windows

# Rebuild from scratch
mkdir build
cd build
cmake ..
```

**Partial Clean** (keeps CMake cache):
```bash
# Linux
make clean

# Windows
cmake --build . --target clean
```

---

## Related Documentation

- [Main README](../README.md)
- [Architecture Documentation](ARCHITECTURE.md)
- [Hardware Setup Guide](HARDWARE_SETUP.md)
- [State Machine Documentation](STATE_MACHINE.md)
