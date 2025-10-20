//
// SharedMemoryManager.h - Manages shared memory communication with Python GUI
//

#ifndef SHARED_MEMORY_MANAGER_H
#define SHARED_MEMORY_MANAGER_H

#include <string>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#endif

// Shared memory structures
struct MotorCommand {
    double target_x;
    double target_y; 
    double target_z;
    bool exit;
    char padding[6];
};

struct StatusUpdate {
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

    bool clearMotorCommand();
    
    // Status writing
    bool writeStatusUpdate(double x, double y, double z, const std::string& status);
    
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