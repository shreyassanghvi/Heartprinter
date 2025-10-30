//
// SharedMemoryManager.h - Manages shared memory communication with Python GUI
//

#ifndef SHARED_MEMORY_MANAGER_H
#define SHARED_MEMORY_MANAGER_H

#include <string>
#include <functional>
#include "../Trackstar/ATC3DG.h"

#ifdef _WIN32
#include <windows.h>
#endif

// Shared memory structures
struct MotorCommand {
    double target_x;
    double target_y;
    double target_z;
    bool execute;
    bool exit;
    char padding[5];
};

struct StatusUpdate {
    double base1_x;
    double base1_y;
    double base1_z;
    double base2_x;
    double base2_y;
    double base2_z;
    double base3_x;
    double base3_y;
    double base3_z;
    double current_x;
    double current_y;
    double current_z;
    char status[6];
    char padding[2];
};

class SharedMemoryManager {
public:
    // Constructor/Destructor
    SharedMemoryManager();
    ~SharedMemoryManager();
    
    // Non-copyable, non-movable
    SharedMemoryManager(const SharedMemoryManager&) = delete;
    SharedMemoryManager& operator=(const SharedMemoryManager&) = delete;
    SharedMemoryManager(SharedMemoryManager&&) = delete;
    SharedMemoryManager& operator=(SharedMemoryManager&&) = delete;
    
    // Main interface
    bool initialize();
    void cleanup();
    
    // Command reading
    bool readMotorCommand(MotorCommand& command);

    bool writeMotorCommand(const MotorCommand& command);
    
    // Status writing
    bool writeStatusUpdate(const DOUBLE_POSITION_ANGLES_RECORD& currentPos,
                          const DOUBLE_POSITION_ANGLES_RECORD basePositions[3],
                          const std::string& status);
    
    // Status
    bool isInitialized() const { return initialized; }
    
private:
    bool initialized = false;
    
#ifdef _WIN32
    // Windows shared memory handles
    HANDLE hMotorCommandMapFile = NULL;
    MotorCommand* pMotorCommandSharedData = nullptr;
    HANDLE hStatusUpdateMapFile = NULL;
    StatusUpdate* pStatusUpdateSharedData = nullptr;
    
    // Shared memory names and sizes
    static constexpr const char* SHM_MOTOR_COMMAND_NAME = "Local\\PyToCPP";
    static constexpr const char* SHM_STATUS_UPDATE_NAME = "Local\\CPPToPy";
    static constexpr size_t SHM_MOTOR_COMMAND_SIZE = sizeof(MotorCommand);
    static constexpr size_t SHM_STATUS_UPDATE_SIZE = sizeof(StatusUpdate);
#endif
    
    // Internal initialization methods
    bool initializeCommandMemory();
    bool initializeStatusMemory();
    void cleanupCommandMemory();
    void cleanupStatusMemory();
};

#endif // SHARED_MEMORY_MANAGER_H