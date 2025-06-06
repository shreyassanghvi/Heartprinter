cmake_minimum_required(VERSION 3.10)

project(system_run)

# Set C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Platform and architecture checks
if(WIN32)
    if(NOT CMAKE_SIZEOF_VOID_P EQUAL 8)
        message(FATAL_ERROR "Only 64-bit Windows builds are supported. Please use a 64-bit generator (e.g., -A x64).")
    endif()
    add_definitions(-DWIN32 -D_WIN32_WINNT=0x0501)
    if(MINGW)
        message(STATUS "Building DynamixelSDK with MinGW")
    endif()
elseif(UNIX AND NOT APPLE)
    add_definitions(-DLINUX -D_GNU_SOURCE)
else()
    message(FATAL_ERROR "Unsupported platform. Only 64-bit Windows and Linux are supported.")
endif()

# Compiler options
add_compile_options(-O2 -O3 -fPIC -pthread -Wall -m64 -g -Wmaybe-uninitialized)

# === Include Directories ===
include_directories(
        ${DIR_DXL}/include/dynamixel_sdk
        ${DIR_DXL}/include/custom
        "C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/"
        ${CMAKE_CURRENT_SOURCE_DIR}/c++/include
        ${CMAKE_CURRENT_SOURCE_DIR}/c++/include/dynamixel_sdk
)

# === Source Files ===
file(GLOB SOURCES "src/custom/*.cpp")
file(GLOB_RECURSE CPP_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/src/dynamixel_sdk/*.cpp")

# === DynamixelSDK Libraries ===
if(NOT TARGET dynamixel_sdk)
    add_library(dynamixel_sdk SHARED ${CPP_SOURCES})
endif()

if(NOT TARGET dynamixel_sdk_static)
    add_library(dynamixel_sdk_static STATIC ${CPP_SOURCES})
endif()

target_include_directories(dynamixel_sdk_static PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/c++/include
        ${CMAKE_CURRENT_SOURCE_DIR}/c++/include/dynamixel_sdk
)

# === Platform-specific Linking and Library Naming ===
if(WIN32)
    target_link_libraries(dynamixel_sdk ws2_32 winmm)
    target_link_libraries(dynamixel_sdk_static ws2_32 winmm)
    set_target_properties(dynamixel_sdk PROPERTIES OUTPUT_NAME "dxl_x64_cpp")
    set_target_properties(dynamixel_sdk_static PROPERTIES OUTPUT_NAME "dxl_x64_cpp_static")

    # Import the ATC3DG64 library
    add_library(ATC3DG64 SHARED IMPORTED GLOBAL)
    set_target_properties(ATC3DG64 PROPERTIES
            IMPORTED_LOCATION "C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/ATC3DG64.dll"
            IMPORTED_IMPLIB   "C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/ATC3DG64.lib"
            INTERFACE_INCLUDE_DIRECTORIES "C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/"
    )
else()
    target_link_libraries(dynamixel_sdk rt)
    target_link_libraries(dynamixel_sdk_static rt)
endif()

# === Main Executable ===
add_executable(${PROJECT_NAME} ${SOURCES})

if(WIN32)
    target_link_libraries(${PROJECT_NAME}
            dynamixel_sdk
            ATC3DG64
    )
else()
    target_link_libraries(${PROJECT_NAME}
            dynamixel_sdk
    )
endif()

set_target_properties(${PROJECT_NAME} PROPERTIES OUTPUT_NAME "system.run")

# === Copy DLL if WIN64 ===
if(WIN32)
    add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "C:/Program Files (x86)/Northern Digital Inc/3D Guidance driveBAY/3DG API Developer/3DG API/ATC3DG64.dll"
            $<TARGET_FILE_DIR:${PROJECT_NAME}>
            COMMENT "Copying ATC3DG64.dll to output directory"
    )
endif()

# === Installation Rules for DynamixelSDK ===
install(TARGETS dynamixel_sdk dynamixel_sdk_static
        LIBRARY DESTINATION lib
        ARCHIVE DESTINATION lib
        RUNTIME DESTINATION bin
)

install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/dynamixel_sdk/ DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/custom/ DESTINATION include
        FILES_MATCHING PATTERN "*.h"
)
