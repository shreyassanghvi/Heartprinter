//
// SharedMemoryManager.cpp - Implementation of shared memory communication with Python GUI
//

#include "../../include/custom/SharedMemoryManager.h"
#include <spdlog/spdlog.h>
#include <cstring>
#include <algorithm>
#define _USE_MATH_DEFINES
#include <math.h>

// Constructor
SharedMemoryManager::SharedMemoryManager() {
    spdlog::debug("SharedMemoryManager created");
}

// Destructor
SharedMemoryManager::~SharedMemoryManager() {
    try {
        cleanup();
    } catch (const std::exception& e) {
        spdlog::error("Exception during SharedMemoryManager destruction: {}", e.what());
    }
}

// Initialize shared memory
bool SharedMemoryManager::initialize() {
    if (initialized) {
        spdlog::debug("SharedMemoryManager already initialized");
        return true;
    }
    
    spdlog::info("Initializing shared memory communication...");
    
    try {
        // Initialize both command and status memory
        if (!initializeCommandMemory()) {
            spdlog::error("Failed to initialize command shared memory");
            return false;
        }
        
        if (!initializeStatusMemory()) {
            spdlog::error("Failed to initialize status shared memory");
            cleanupCommandMemory();
            return false;
        }
        
        initialized = true;
        spdlog::info("Shared memory communication initialized successfully");
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during shared memory initialization: {}", e.what());
        cleanup();
        return false;
    }
}

// Initialize command memory (read from Python)
bool SharedMemoryManager::initializeCommandMemory() {
    try {
        // Create shared memory for motor commands (read from Python)
        hMotorCommandMapFile = CreateFileMapping(
            INVALID_HANDLE_VALUE,
            NULL,
            PAGE_READWRITE,
            0,
            SHM_MOTOR_COMMAND_SIZE,
            SHM_MOTOR_COMMAND_NAME);
        
        if (hMotorCommandMapFile == NULL) {
            spdlog::error("CreateFileMapping failed for motor commands: {}", GetLastError());
            return false;
        }
        
        pMotorCommandSharedData = (MotorCommand*)MapViewOfFile(
            hMotorCommandMapFile,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            SHM_MOTOR_COMMAND_SIZE);
        
        if (pMotorCommandSharedData == NULL) {
            spdlog::error("MapViewOfFile failed for motor commands: {}", GetLastError());
            CloseHandle(hMotorCommandMapFile);
            hMotorCommandMapFile = NULL;
            return false;
        }
        
        spdlog::debug("Command shared memory initialized");
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during command memory initialization: {}", e.what());
        return false;
    }
}

// Initialize status memory (write to Python)
bool SharedMemoryManager::initializeStatusMemory() {
    try {
        // Create shared memory for status updates (write to Python)
        hStatusUpdateMapFile = CreateFileMapping(
            INVALID_HANDLE_VALUE,
            NULL,
            PAGE_READWRITE,
            0,
            SHM_STATUS_UPDATE_SIZE,
            SHM_STATUS_UPDATE_NAME);
        
        if (hStatusUpdateMapFile == NULL) {
            spdlog::error("CreateFileMapping failed for status updates: {}", GetLastError());
            return false;
        }
        
        pStatusUpdateSharedData = (StatusUpdate*)MapViewOfFile(
            hStatusUpdateMapFile,
            FILE_MAP_ALL_ACCESS,
            0,
            0,
            SHM_STATUS_UPDATE_SIZE);
        
        if (pStatusUpdateSharedData == NULL) {
            spdlog::error("MapViewOfFile failed for status updates: {}", GetLastError());
            CloseHandle(hStatusUpdateMapFile);
            hStatusUpdateMapFile = NULL;
            return false;
        }
        
        spdlog::debug("Status shared memory initialized");
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during status memory initialization: {}", e.what());
        return false;
    }
}

// Cleanup all shared memory
void SharedMemoryManager::cleanup() {
    if (!initialized) {
        return;
    }
    
    spdlog::debug("Cleaning up shared memory...");
    
    try {
        cleanupStatusMemory();
        cleanupCommandMemory();
        
        initialized = false;
        spdlog::debug("Shared memory cleanup complete");
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during shared memory cleanup: {}", e.what());
    }
}

// Cleanup command memory
void SharedMemoryManager::cleanupCommandMemory() {
    if (pMotorCommandSharedData) {
        UnmapViewOfFile(pMotorCommandSharedData);
        pMotorCommandSharedData = nullptr;
    }
    if (hMotorCommandMapFile) {
        CloseHandle(hMotorCommandMapFile);
        hMotorCommandMapFile = NULL;
    }
}

// Cleanup status memory
void SharedMemoryManager::cleanupStatusMemory() {
    if (pStatusUpdateSharedData) {
        UnmapViewOfFile(pStatusUpdateSharedData);
        pStatusUpdateSharedData = nullptr;
    }
    if (hStatusUpdateMapFile) {
        CloseHandle(hStatusUpdateMapFile);
        hStatusUpdateMapFile = NULL;
    }
}

// Read motor command from shared memory
bool SharedMemoryManager::readMotorCommand(MotorCommand& command) {
    if (!initialized) {
        return false;
    }

    if (!pMotorCommandSharedData) {
        return false;
    }

    try {
        memcpy(&command, pMotorCommandSharedData, sizeof(MotorCommand));
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Exception reading motor command: {}", e.what());
        return false;
    }
}

// Write motor command to shared memory
bool SharedMemoryManager::writeMotorCommand(const MotorCommand& command) {
    if (!initialized) {
        return false;
    }

    if (!pMotorCommandSharedData) {
        return false;
    }

    try {
        memcpy(pMotorCommandSharedData, &command, sizeof(MotorCommand));
        spdlog::debug("Motor command written to shared memory (execute={}, exit={})",
                     command.execute, command.exit);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Exception writing motor command: {}", e.what());
        return false;
    }
}

// Write status update to shared memory
bool SharedMemoryManager::writeStatusUpdate(const DOUBLE_POSITION_ANGLES_RECORD& currentPos,
                                           const DOUBLE_POSITION_ANGLES_RECORD basePositions[3],
                                           const std::string& status) {
    if (!initialized) {
        return false;
    }

    if (!pStatusUpdateSharedData) {
        return false;
    }

    try {
        StatusUpdate msg;

        // Base 1 position
        msg.base1_x = basePositions[0].x;
        msg.base1_y = basePositions[0].y;
        msg.base1_z = basePositions[0].z;

        // Base 2 position
        msg.base2_x = basePositions[1].x;
        msg.base2_y = basePositions[1].y;
        msg.base2_z = basePositions[1].z;

        // Base 3 position
        msg.base3_x = basePositions[2].x;
        msg.base3_y = basePositions[2].y;
        msg.base3_z = basePositions[2].z;

        // Current position (moving base)
        msg.current_x = currentPos.x;
        msg.current_y = currentPos.y;
        msg.current_z = currentPos.z;

        // Status string
        size_t copy_len = std::min(status.length(), size_t(5));
        strncpy_s(msg.status, sizeof(msg.status), status.c_str(), copy_len);
        msg.status[copy_len] = '\0';

        memset(msg.padding, 0, sizeof(msg.padding));
        memcpy(pStatusUpdateSharedData, &msg, sizeof(StatusUpdate));

        spdlog::debug("Status update written: Base1({:.2f}, {:.2f}, {:.2f}) Base2({:.2f}, {:.2f}, {:.2f}) Base3({:.2f}, {:.2f}, {:.2f}) Current({:.2f}, {:.2f}, {:.2f}) - {}",
                     basePositions[0].x, basePositions[0].y, basePositions[0].z,
                     basePositions[1].x, basePositions[1].y, basePositions[1].z,
                     basePositions[2].x, basePositions[2].y, basePositions[2].z,
                     currentPos.x, currentPos.y, currentPos.z,
                     status);
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Exception writing status update: {}", e.what());
        return false;
    }
}