//
// SystemController.h - Simplified Heart Printer System Controller
//

#ifndef SYSTEM_CONTROLLER_H
#define SYSTEM_CONTROLLER_H

#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#endif

// Forward declarations for existing hardware classes
class Motor;
class CTrackstar;
class SharedMemoryManager;
struct DAQSystem;
struct DigitalConfig;
struct AnalogConfig;

// System States
enum class States {
    START,
    INITIALIZING,
    READY,
    RUNNING,
    MOVING,
    ERROR,
    END,
    CLEANUP
};

// System Errors
enum class SystemError {
    NONE,
    MOTOR_INIT_FAILED,
    TRACKER_INIT_FAILED,
    DAQ_INIT_FAILED,
    COMMUNICATION_FAILED,
    SAFETY_VIOLATION,
    EMERGENCY_STOP
};

// Forward declaration for tracking data structure
struct DOUBLE_POSITION_ANGLES_RECORD;

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

// System configuration
struct SystemConfig {
    std::string deviceName = "/dev/ttyUSB0";
    int baudRate = 57600;
    int recordCount = 1000;
    bool enableSafetyChecks = true;
};

// System status
struct SystemStatus {
    States currentState = States::START;
    DOUBLE_POSITION_ANGLES_RECORD currentPosition = {};
    bool motorsEnabled = false;
    bool trackingActive = false;
    bool daqActive = false;
    std::string lastError;
};

// Custom exception class
class SystemException : public std::runtime_error {
private:
    SystemError errorCode;
    
public:
    SystemException(SystemError code, const std::string& message)
        : std::runtime_error(message), errorCode(code) {}
    
    SystemError getErrorCode() const { return errorCode; }
};

// Main SystemController class
class SystemController {
private:
    // Configuration
    SystemConfig config;
    
    // Current state
    States currentState = States::START;
    DOUBLE_POSITION_ANGLES_RECORD currentPosition = {};
    std::string lastErrorMessage;
    bool initialized = false;
    
    // Hardware components - direct integration with existing classes
    std::vector<Motor> motors;
    std::unique_ptr<CTrackstar> tracker;
    std::unique_ptr<DAQSystem> daqSystem;
    std::unique_ptr<SharedMemoryManager> sharedMemory;
    
    // Motor control variables
    std::vector<int> motorDestinations;
    int minPos = 0;
    int maxPos = 0;
    int neutralPos = 0;
    
    // Internal methods
    bool initializeHardware();
    bool initializeMotors();
    bool initializeTracker();
    bool initializeDAQ();
    void cleanupDAQ();
    States processStateTransition(States current);
    bool performSafetyCheck(const DOUBLE_POSITION_ANGLES_RECORD& targetPos) const;
    void handleError(const SystemException& e);
    
    // Shared memory methods
    std::string currentStateToString() const;

    // Motor control methods from StateController
    bool readSharedMemoryCommand(MotorCommand& cmd);
    void calculateMotorPositionsFromCommand(const MotorCommand& cmd);
    void calculateMotorPositionsFromTracking();
    bool moveMotorPositions();
    bool readMotorPositions();
    
public:
    // Constructor/Destructor - RAII
    explicit SystemController(const SystemConfig& config = SystemConfig{});
    ~SystemController();
    
    // Non-copyable, non-movable
    SystemController(const SystemController&) = delete;
    SystemController& operator=(const SystemController&) = delete;
    SystemController(SystemController&&) = delete;
    SystemController& operator=(SystemController&&) = delete;
    
    // Main interface
    bool initialize();
    void run();
    void shutdown();
    
    // Motor control
    bool disableMotors();
    bool moveToTarget(const DOUBLE_POSITION_ANGLES_RECORD& target);
    
    // Tracking control
    bool stopTracking();
    
    // DAQ control
    bool setLEDState(int led, bool state);
    bool setAllLEDs(bool state);
    bool stopAnalogAcquisition();
    
    // Status and monitoring
    SystemStatus getSystemStatus() const;
    bool isInitialized() const { return initialized; }
    States getCurrentState() const { return currentState; }
    const DOUBLE_POSITION_ANGLES_RECORD& getCurrentPosition() const { return currentPosition; }
    std::string getLastError() const { return lastErrorMessage; }
    bool isInSafeZone(double x, double y, double z) const;
    
    // Configuration
    void setConfiguration(const SystemConfig& newConfig);
    SystemConfig getConfiguration() const { return config; }
};

#endif // SYSTEM_CONTROLLER_H