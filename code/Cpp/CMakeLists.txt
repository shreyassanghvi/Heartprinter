cmake_minimum_required(VERSION 3.10)

# Set the project name
project(DXL_Protocol_Example)

# Set the target executable name
set(TARGET system.run)

# Specify the directories
set(DIR_DXL ..)
set(DIR_OBJS .objects)

# Compiler options
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2 -O3 -DLINUX -D_GNU_SOURCE -fPIC -pthread -Wall -m64 -g")

# Include directories
include_directories(
        ${DIR_DXL}/include/dynamixel_sdk
)
include_directories(
        ${DIR_DXL}/include/custom
)
include_directories(
        /opt/3DGuidance.Rev.E.64/3DGuidanceAPI
)
# Source files
set(SOURCES
        ./src/custom/Motor.cpp
        ./src/custom/init.cpp
        # ./src/custom/Sample.cpp
        # *** OTHER SOURCES GO HERE ***
)

# Create an object directory if it doesn't exist (CMake handles this differently)
file(MAKE_DIRECTORY ${CMAKE_BINARY_DIR}/${DIR_OBJS})

# Add the executable target
add_executable(${TARGET} ${SOURCES})

# Link libraries
target_link_libraries(${TARGET}
        dxl_x64_cpp
        rt
        /usr/lib64/ATC3DGlib64.so
)

# Custom clean command (optional)
set_property(DIRECTORY APPEND PROPERTY ADDITIONAL_MAKE_CLEAN_FILES ${TARGET} ${DIR_OBJS})
