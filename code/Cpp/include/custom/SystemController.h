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
#include <mutex>
#include <atomic>

#ifdef _WIN32
#include <windows.h>
#endif

#include "../Trackstar/ATC3DG.h"
#include "../Dynamixel_SDK/dynamixel_sdk.h"

// Forward declarations for existing hardware classes
class Motor;
class CSystem;
class SharedMemoryManager;
struct DAQSystem;
struct DigitalConfig;
struct AnalogConfig;

// System States
enum class States {
    START,
    INITIALIZING,
    CALIBRATION,
    READY,
    RUNNING,
    MOVING,
    ERR,
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
// struct DOUBLE_POSITION_ANGLES_RECORD;

// Shared memory structures
struct MotorCommand;

struct StatusUpdate;

// System configuration
struct SystemConfig {
    // Motor configuration
    std::string deviceName = "/dev/ttyUSB0";
    int baudRate = 57600;
    int recordCount = 1000;

    // Safety configuration
    bool enableSafetyChecks = true;
    double maxLoadVoltage = -0.5;      // Maximum safe load cell voltage
    double minLoadVoltage = -1.0;        // Minimum expected load cell voltage

    // Tension adjustment configuration
    int tensionAdjustmentSteps = 2;     // Number of motor steps to adjust tension (1-2 recommended)

    // Tracking configuration
    double trackingDeadband = 300.0;    // Deadband for tracking position (mm)

    // Calibration configuration
    int calibrationStabilityReads = 10;      // Number of consistent reads to detect base arrival
    double calibrationPositionTolerance = 5.0; // Position tolerance in motor units for stability detection
    int calibrationMovementSteps = 100;     // Number of motor steps to move during calibration base detection
};

// System status
struct SystemStatus {
    States currentState = States::START;
    DOUBLE_POSITION_ANGLES_RECORD currentPosition;
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

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;
    dynamixel::GroupSyncWrite groupSyncWrite;
    dynamixel::GroupSyncRead groupSyncRead;
    
    // Hardware components - direct integration with existing classes
    std::vector<Motor> motors;
    std::unique_ptr<CSystem> tracker;
    std::unique_ptr<DAQSystem> daqSystem;
    std::unique_ptr<SharedMemoryManager> sharedMemory;
    
    // Motor control variables
    std::vector<int> motorDestinations;
    int minPos = 0;
    int maxPos = 0;
    int neutralPos = 0;

    // DAQ channel averages (thread-safe access from callback)
    mutable std::mutex daqDataMutex;
    std::atomic<bool> daqDataAvailable{false};
    double daqChannelAverages[3] = {0.0, 0.0, 0.0};

    // Calibration data
    DOUBLE_POSITION_ANGLES_RECORD staticBasePositions[3] = {};  // Positions of the 3 static bases
    DOUBLE_POSITION_ANGLES_RECORD centroidPosition = {};        // Calculated centroid position

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
    bool adjustTensionBasedOnLoadCells();

    // Calibration methods
    bool validateProbes();
    bool performCalibration();
    bool moveToBase(int baseIndex);
    bool detectBaseArrival(int motorIndex);
    void calculateCentroid();
    void setMotorDestinationsForTarget(DOUBLE_POSITION_ANGLES_RECORD& targetPos);
    
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
    
    // Tracking control
    bool stopTracking();
    
    // DAQ control
    bool setLEDState(int led, bool state);
    bool setAllLEDs(bool state);
    bool stopAnalogAcquisition();
    bool getDAQChannelAverages(double averages[3]) const;

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