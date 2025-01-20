cmake_minimum_required(VERSION 3.10)

# Project name
project(system_run)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
#set(CLLIBS  /usr/lib64/ATC3DGlib64.so)
# Compiler options
add_compile_options(-O2 -O3 -DLINUX -D_GNU_SOURCE -fPIC -pthread -Wall -m64 -g)

link_directories(/usr/lib64/)
# Include directories
include_directories(
        ${CMAKE_SOURCE_DIR}/../include/dynamixel_sdk
        ${CMAKE_SOURCE_DIR}/../include/custom
        /opt/3DGuidance.Rev.E.64/3DGuidanceAPI/
)

# Source files
set(SOURCES
        src/custom/Motor.cpp
        src/custom/init.cpp
        # src/custom/Sample.cpp  # Commented out as in the original Makefile
)

# Create executable
add_executable(${PROJECT_NAME} ${SOURCES})

add_library(ATC3DGlib64 SHARED IMPORTED)
set_target_properties(ATC3DGlib64 PROPERTIES IMPORTED_LOCATION /usr/lib64/ATC3DGlib64.so)

target_link_libraries(${PROJECT_NAME}
        dxl_x64_cpp
        rt
        ATC3DGlib64
)

# Set output name
set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "system.run")
