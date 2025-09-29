//
// SystemController.cpp - Implementation of the Simplified Heart Printer System Controller
//

#include "../../include/custom/SystemController.h"
#include "../../include/custom/Motor.h"
#include "../../include/custom/CTrackstar.h"
#include "../../include/custom/NIDAQ.h"
#include "../../include/custom/SharedMemoryManager.h"
#include "../../include/Trackstar/ATC3DG.h"
#include "../Dynamixel_SDK/dynamixel_sdk.h"

#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>
#include <cstring>

#ifdef _WIN32
#include <windows.h>
#endif

// Motor system constants
#define MOTOR_CNT 3

// Dynamixel SDK global variables (moved from Init.cpp)
static dynamixel::PortHandler *portHandler = nullptr;
static dynamixel::PacketHandler *packetHandler = nullptr;
static dynamixel::GroupSyncWrite *groupSyncWrite = nullptr;
static dynamixel::GroupSyncRead *groupSyncRead = nullptr;

// Constructor
SystemController::SystemController(const SystemConfig& config)
    : config(config), currentState(States::START) {
    spdlog::info("SystemController created with device: {}", config.deviceName);

    // Initialize motor control variables
    minPos = DXL_MINIMUM_POSITION_VALUE;
    maxPos = static_cast<int>(DXL_MAXIMUM_POSITION_VALUE * 1.5);
    neutralPos = (minPos + maxPos) / 2;
    motorDestinations.resize(MOTOR_CNT, 0);

    // Create shared memory manager
    sharedMemory = std::make_unique<SharedMemoryManager>();
}

// Destructor - RAII cleanup
SystemController::~SystemController() {
    try {
        if (currentState != States::END) {
            shutdown();
        }
        if (sharedMemory) {
            sharedMemory->cleanup();
        }
    } catch (const std::exception& e) {
        spdlog::error("Exception during SystemController destruction: {}", e.what());
    }
}

// Initialize all hardware components
bool SystemController::initialize() {
    try {
        spdlog::info("Initializing Heart Printer System...");
        currentState = States::INITIALIZING;
        
        // Initialize shared memory first
        if (sharedMemory && !sharedMemory->initialize()) {
            spdlog::warn("Failed to initialize shared memory - continuing without it");
        }

        sharedMemory->writeStatusUpdate(currentPosition.x, currentPosition.y, currentPosition.z, currentStateToString());

        if (!initializeHardware()) {
            handleError(SystemException(SystemError::MOTOR_INIT_FAILED, 
                                      "Failed to initialize hardware"));
            return false;
        }
        currentState = States::READY;
        initialized = true;

        sharedMemory->writeStatusUpdate(currentPosition.x, currentPosition.y, currentPosition.z, currentStateToString());
        spdlog::info("System initialization complete - Ready for operation");
        return true;
        
    } catch (const SystemException& e) {
        handleError(e);
        return false;
    } catch (const std::exception& e) {
        handleError(SystemException(SystemError::COMMUNICATION_FAILED, e.what()));
        return false;
    }
}

// Initialize hardware components
bool SystemController::initializeHardware() {
    // Initialize DAQ system
    if (!initializeDAQ()) {
        throw SystemException(SystemError::DAQ_INIT_FAILED,
                            "Failed to initialize DAQ system");
    }

    // Initialize motors
    if (!initializeMotors()) {
        throw SystemException(SystemError::MOTOR_INIT_FAILED, 
                            "Failed to initialize motors");
    }
    
    // Initialize tracking system
    if (!initializeTracker()) {
        throw SystemException(SystemError::TRACKER_INIT_FAILED, 
                            "Failed to initialize TrackStar system");
    }
    
    spdlog::info("All hardware components initialized successfully");
    return true;
}

// Initialize DAQ system
bool SystemController::initializeDAQ() {
    spdlog::info("Initializing DAQ system...");

    try {
        // Configure DAQ system
        DigitalConfig digitalConf;
        digitalConf.device = "Dev1";
        digitalConf.channel = "PFI0:2";

        AnalogConfig analogConf;
        analogConf.device = "Dev1";
        analogConf.channel = "ai0:2";
        analogConf.minVoltage = 0.0;
        analogConf.maxVoltage = 10.0;
        analogConf.sampleRate = 10000.0;
        analogConf.samplesPerCallback = 1000;

        // Create DAQ system
        daqSystem = std::make_unique<DAQSystem>();
        *daqSystem = initDAQSystem(digitalConf, analogConf,
           [](double* data, uInt32 numSamples) {
               // Static callback wrapper - would need instance access
               try {
                       if (!data || numSamples == 0) {
                           spdlog::warn("Invalid data or sample count in DataHandler!");
                           return;
                       }
                       // printf("Test\n");
                       // printf("Received %u samples. First: %.3f\n", numSamples, data[0]);
                       // if (data == nullptr) {
                       //     printf("No data\n");
                       //     return;
                       // }
                       std::vector vData(data, data + numSamples);
                       spdlog::info("DAQ - Sample Count: {} - Received Data[0] - {:8.5f}",
                                    numSamples,
                                    data[0]);
                   } catch (std::exception &e) {
                       spdlog::error("Error: {}", e.what());

                       state = ERROR;
                   }
           },
           [](const char* errorMessage) {
               spdlog::error("DAQ Error: {}", errorMessage);
               state = ERROR;
           }
        );

        if (!daqSystem->initialized) {
            spdlog::error("Failed to initialize DAQ system");
            return false;
        }

        // Test LED functionality
        if (!setAllLEDs(true)) {
            spdlog::error("Failed to set all LEDs off");
            return false;
        }
        if (!setAllLEDs(false) {
            spdlog::error("Failed to set all LEDs off");
            return false;
        }

        // TODO: maybe move into a separate function
        spdlog::warn("TODO: Initializing load cell...");
        if (DAQStart(daqSystem.analogHandle) != 0){
            return false;
        }

        spdlog::info("DAQ system initialized successfully");
        return true;

    } catch (const std::exception& e) {
        spdlog::error("Exception during DAQ initialization: {}", e.what());
        return false;
    }
}

// Initialize motors using direct Motor objects (self-contained)
bool SystemController::initializeMotors() {
    spdlog::info("Initializing motor controllers...");
    
    try {
        // Initialize Dynamixel SDK components
        portHandler = dynamixel::PortHandler::getPortHandler(config.deviceName.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
        groupSyncWrite = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
        groupSyncRead = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
        
        // Open port
        if (!portHandler->openPort()) {
            spdlog::error("Failed to open the port: {}", config.deviceName);
            return false;
        }
        spdlog::info("Succeeded to open the port: {}", config.deviceName);
        
        // Set port baudrate
        if (!portHandler->setBaudRate(config.baudRate)) {
            spdlog::error("Failed to change the baudrate to: {}", config.baudRate);
            return false;
        }
        spdlog::info("Succeeded to change the baudrate to: {}", config.baudRate);
        
        // Create motor instances for the 3 motors
        motors.clear();
        motors.reserve(MOTOR_CNT);
        
        for (int i = 0; i < MOTOR_CNT; i++) {
            // Create Motor directly with ID (1-based indexing)
            motors.emplace_back(i + 1);
            
            // Initialize each motor (logic from initMotors function in Init.cpp)
            // Set motor operation mode to extended position control
            if (motors.back().setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE) != 0) {
                spdlog::error("Failed to set operation mode for motor {}", i + 1);
                return false;
            }
            
            // Enable motor torque
            if (motors.back().enableTorque(packetHandler, portHandler) != 0) {
                spdlog::error("Failed to enable torque for motor {}", i + 1);
                return false;
            }
            
            // Set initial position to minimum
            if (!motors.back().setMotorDestination(groupSyncWrite, DXL_MINIMUM_POSITION_VALUE)) {
                spdlog::error("Failed to set initial position for motor {}", i);
                return false;
            }
        }
        
        // Execute initial position setting
        int dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("Failed to execute initial motor positioning: {}", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        
        groupSyncWrite->clearParam();
        spdlog::info("Motor controllers initialized successfully - torque enabled");
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during motor initialization: {}", e.what());
        return false;
    }
}

// Initialize tracker
bool SystemController::initializeTracker() {
    spdlog::info("Initializing tracking system...");
    
    try {
        tracker = std::make_unique<CTrackstar>();
        
        // Initialize TrackStar system using existing functions
        initTxRx();
        
        int sensorCount = getConnectedSensors();
        if (sensorCount <= 0) {
            spdlog::error("No sensors detected in TrackStar system");
            return false;
        }
        
        spdlog::info("TrackStar system initialized with {} sensors", sensorCount);
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during tracker initialization: {}", e.what());
        return false;
    }
}

// Main control loop
void SystemController::run() {
    if (currentState != States::READY) {
        spdlog::error("System not ready - current state: {}", 
                     static_cast<int>(currentState));
        return;
    }

    // Should get us from READY to RUNNING
    currentState = processStateTransition(currentState);
    try {
        spdlog::info("Starting main control loop...");

        int dataCount = 0;
        const int MAX_RECORDS = config.recordCount;
        
        // Main control loop - using StateController pattern
        // Only should go into this when RUNNING or MOVING.
        while ((currentState == States::RUNNING || currentState == States::MOVING) && dataCount < MAX_RECORDS) {
            // Process state transitions
            States newState = processStateTransition(currentState);
            if (newState != currentState) {
                spdlog::info("{} -> {}", currentStateToString(currentState), currentStateToString(newState));
                currentState = newState;
            }
            sharedMemory->writeStatusUpdate(currentPosition.x, currentPosition.y, currentPosition.z, currentStateToString());

            // Break if we're no longer in a valid running state
            if (currentState != States::RUNNING && currentState != States::MOVING) {
                break;
            }
            
            // Read tracking data
            try {
                currentPosition = readATI(0); // Read from first sensor directly

                dataCount++;

                // Log position data
                spdlog::info("TS: [{:3d}] {:8.3f} {:8.3f} {:8.3f} : {:8.3f} {:8.3f} {:8.3f}",
                           dataCount,
                           currentPosition.x, currentPosition.y, currentPosition.z,
                           currentPosition.a, currentPosition.e, currentPosition.r);

            } catch (const std::exception& e) {
                spdlog::error("Failed to read tracking data: {}", e.what());
                handleError(SystemException(SystemError::TRACKER_INIT_FAILED, "Tracking read failed"));
                break;
            }

            // Read from shared memory
            MotorCommand sharedCmd;
            bool commandAvailable = readSharedMemoryCommand(sharedCmd);

            // If we have an exit flag, then let's exit appropriately
            if (commandAvailable && sharedCmd.exit) {
                spdlog::info("Exit command received from shared memory");
                currentState = States::CLEANUP;
            }

            // If we are in the RUNNING state:
            if (currentState == States::RUNNING) {
                // If the commandProcessed exists, then let's set motorpositions based on that
                if (commandAvailable) {
                    calculateMotorPositionsFromCommand(sharedCmd);
                } else {
                    // Otherwise, let's set motorpositions based on the tracking function
                    calculateMotorPositionsFromTracking();
                }

                // Send new motor commands
                if (!moveMotorPositions()) {
                    spdlog::error("Failed to move motors");
                    handleError(SystemException(SystemError::MOTOR_INIT_FAILED, "Motor movement failed"));
                    break;
                }
            }
            // If we are in the MOVING state, just continue
            
            // The state transition logic will handle moving to MOVING state
            // if motors haven't reached their targets yet
            
            // Small delay to prevent tight loop
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
        
        spdlog::info("Control loop completed - collected {} records", dataCount);
        
    } catch (const SystemException& e) {
        handleError(e);
    } catch (const std::exception& e) {
        handleError(SystemException(SystemError::COMMUNICATION_FAILED, e.what()));
    }
}

// Process state transitions with proper error handling
States SystemController::processStateTransition(States current) {
    switch (current) {
        case States::START:
            return States::INITIALIZING;

        case States::INITIALIZING:
            return States::READY;

        case States::READY:
            return States::RUNNING;

        case States::RUNNING:
            // Check for safety conditions
            if (!performSafetyCheck(currentPosition)) {
                spdlog::warn("Safety check failed - entering error state");
                return States::ERROR;
            }

            // Check if motors are moving
            if (!readMotorPositions()) {
                return States::MOVING;
            }

            return States::RUNNING;

        case States::MOVING:
            // Check if motors have reached their targets
            if (readMotorPositions()) {
                spdlog::debug("Motors reached target positions");
                return States::RUNNING;
            }

            // Check for safety conditions while moving
            if (!performSafetyCheck(currentPosition)) {
                spdlog::warn("Safety check failed while moving - stopping");
                return States::ERROR;
            }

            return States::MOVING;

        case States::ERROR:
            return States::CLEANUP;

        case States::CLEANUP:
            return States::END;

        case States::END:
            return States::END;

        default:
            spdlog::error("Unknown state: {}", static_cast<int>(current));
            return States::ERROR;
    }
}

// Safety checks
bool SystemController::performSafetyCheck(const DOUBLE_POSITION_ANGLES_RECORD& targetPos) const {
    if (!config.enableSafetyChecks) {
        return true;
    }
    
    // Basic position limits (adjust as needed)
    const double MAX_POSITION = 100.0;  // mm
    const double MIN_POSITION = -100.0; // mm
    
    if (std::abs(targetPos.x) > MAX_POSITION ||
        std::abs(targetPos.y) > MAX_POSITION ||
        std::abs(targetPos.z) > MAX_POSITION) {
        return false;
    }
    
    return true;
}

// Disable motors
bool SystemController::disableMotors() {
    try {
        for (auto& motor : motors) {
            if (motor.disableTorque(packetHandler, portHandler) != 0) {
                spdlog::error("Failed to disable motor {} torque", motor.getMotorID());
            }
        }
        spdlog::info("All motors disabled");
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Exception disabling motors: {}", e.what());
        return false;
    }
}

// Stop tracking
bool SystemController::stopTracking() {
    // TrackStar cleanup would happen in shutdown
    spdlog::info("Tracking stopped");
    return true;
}

// Set LED state
bool SystemController::setLEDState(int led, bool state) {
    if (!daqSystem || !daqSystem->initialized) {
        return false;
    }
    
    try {
        LED ledEnum = static_cast<LED>(led);
        int result = ::setLEDState(daqSystem->digitalTask, ledEnum, state ? LED_ON : LED_OFF);
        return (result == 0);
    } catch (const std::exception& e) {
        spdlog::error("Exception setting LED state: {}", e.what());
        return false;
    }
}

// Set all LEDs
bool SystemController::setAllLEDs(bool state) {
    if (!daqSystem || !daqSystem->initialized) {
        return false;
    }
    
    try {
        int result = ::setAllLEDs(daqSystem->digitalTask, state ? LED_ON : LED_OFF);
        return (result == 0);
    } catch (const std::exception& e) {
        spdlog::error("Exception setting all LEDs: {}", e.what());
        return false;
    }
}

// Stop analog acquisition
bool SystemController::stopAnalogAcquisition() {
    if (!daqSystem || !daqSystem->initialized) {
        return true;
    }
    
    try {
        int result = DAQStop(daqSystem->analogHandle);
        return (result == 0);
    } catch (const std::exception& e) {
        spdlog::error("Exception stopping analog acquisition: {}", e.what());
        return false;
    }
}


// Shutdown system
void SystemController::shutdown() {
    spdlog::info("Shutting down system...");
    currentState = States::CLEANUP;

    sharedMemory->writeStatusUpdate(currentPosition.x, currentPosition.y, currentPosition.z, currentStateToString());
    try {
        // Stop all operations
        stopAnalogAcquisition();
        setAllLEDs(false);
        disableMotors();
        stopTracking();
        
        // Cleanup DAQ system
        cleanupDAQ();
        
        // Cleanup Dynamixel SDK components
        if (portHandler) {
            portHandler->closePort();
        }
        if (groupSyncWrite) {
            delete groupSyncWrite;
            groupSyncWrite = nullptr;
        }
        if (groupSyncRead) {
            delete groupSyncRead;
            groupSyncRead = nullptr;
        }
        
        // Clear hardware components
        motors.clear();
        tracker.reset();
        daqSystem.reset();
        
        initialized = false;
        currentState = States::END;
        spdlog::info("System shutdown complete");
        
    } catch (const std::exception& e) {
        spdlog::error("Error during shutdown: {}", e.what());
    }
}

// Cleanup DAQ system
void SystemController::cleanupDAQ() {
    if (daqSystem) {
        try {
            cleanupDAQSystem(*daqSystem);
        } catch (const std::exception& e) {
            spdlog::error("Exception during DAQ cleanup: {}", e.what());
        }
    }
}

// Handle system errors
void SystemController::handleError(const SystemException& e) {
    currentState = States::ERROR;
    lastErrorMessage = e.what();
    spdlog::error("System error [{}]: {}", 
                 static_cast<int>(e.getErrorCode()), e.what());
}

// Get system status
SystemStatus SystemController::getSystemStatus() const {
    SystemStatus status;
    status.currentState = currentState;
    status.currentPosition = currentPosition;
    status.motorsEnabled = !motors.empty(); // Simplified check
    status.trackingActive = (tracker != nullptr);
    status.daqActive = (daqSystem && daqSystem->initialized);
    status.lastError = lastErrorMessage;
    
    return status;
}

// Check if position is in safe zone
bool SystemController::isInSafeZone(double x, double y, double z) const {
    DOUBLE_POSITION_ANGLES_RECORD pos = {x, y, z, 0.0, 0.0, 0.0};
    return performSafetyCheck(pos);
}

// Move to target position
bool SystemController::moveToTarget(const DOUBLE_POSITION_ANGLES_RECORD& target) {
    if (!performSafetyCheck(target)) {
        spdlog::error("Target position outside safe limits");
        return false;
    }
    
    // This would be implemented with inverse kinematics
    // For now, just update the reference position
    spdlog::info("Moving to target: ({:.2f}, {:.2f}, {:.2f})", 
                target.x, target.y, target.z);
    
    return true;
}

// Set configuration
void SystemController::setConfiguration(const SystemConfig& newConfig) {
    if (initialized) {
        spdlog::warn("Cannot change configuration while system is initialized");
        return;
    }
    
    config = newConfig;
    spdlog::info("Configuration updated");
}


// Convert current state to string
std::string SystemController::currentStateToString() const {
    switch (currentState) {
        case States::START: return "START";
        case States::INITIALIZING: return "INIT";
        case States::READY: return "READY";
        case States::RUNNING: return "RUN";
        case States::MOVING: return "MOVE";
        case States::ERROR: return "ERR";
        case States::END: return "END";
        case States::CLEANUP: return "CLEAN";
        default: return "UNK";
    }
}

// Read shared memory command and update status
bool SystemController::readSharedMemoryCommand(MotorCommand& cmd) {
    if (!sharedMemory) {
        return false;
    }

    // Update status in shared memory
    sharedMemory->writeStatusUpdate(currentPosition.x, currentPosition.y, currentPosition.z, currentStateToString());

    // Try to read shared memory command
    return sharedMemory->readMotorCommand(cmd);
}

// Calculate motor positions from shared memory command
void SystemController::calculateMotorPositionsFromCommand(const MotorCommand& cmd) {
    static const double DEADBAND = 3.0;
    if ((DEADBAND < cmd.target_x < -DEADBAND) ||
        (DEADBAND < cmd.target_y < -DEADBAND) ||
        (DEADBAND < cmd.target_z < -DEADBAND)) {
        spdlog::warn("SHM: Target cannot be reached, outside of DEADBAND.");
    }

    // Convert mm to Dynamixel units using first motor's conversion function
    if (!motors.empty()) {
        motorDestinations[0] = motors[0].mmToDynamixelUnits(cmd.target_x);
        motorDestinations[1] = motors[0].mmToDynamixelUnits(cmd.target_y);
        motorDestinations[2] = motors[0].mmToDynamixelUnits(cmd.target_z);

        spdlog::info("SHM: Target ({:.2f}, {:.2f}, {:.2f}) -> Dest: [{:4d}, {:4d}, {:4d}]",
                    cmd.target_x, cmd.target_y, cmd.target_z,
                    motorDestinations[0], motorDestinations[1], motorDestinations[2]);
    }
}

// Calculate motor positions from tracking data
void SystemController::calculateMotorPositionsFromTracking() {
    static const double DEADBAND = 3.0;
    static const double REF_X = 0.0;
    static const double REF_Y = 0.0;
    static const double REF_Z = 0.0;

    double dx = currentPosition.x - REF_X;
    double dy = currentPosition.y - REF_Y;
    double dz = currentPosition.z - REF_Z;

    // X axis (motor 0): normal logic
    if (dx > DEADBAND) {
        motorDestinations[0] = maxPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), true);
    } else if (dx < -DEADBAND) {
        motorDestinations[0] = minPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), true);
    } else {
        motorDestinations[0] = neutralPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), false);
    }

    // Y axis (motor 1): inverted logic
    if (dy > DEADBAND) {
        motorDestinations[1] = minPos;  // inverted
        setLEDState(static_cast<int>(LED::CENTER_BASE), true);
    } else if (dy < -DEADBAND) {
        motorDestinations[1] = maxPos;  // inverted
        setLEDState(static_cast<int>(LED::CENTER_BASE), true);
    } else {
        motorDestinations[1] = neutralPos;
        setLEDState(static_cast<int>(LED::CENTER_BASE), false);
    }

    // Z axis (motor 2): normal logic
    if (dz > DEADBAND) {
        motorDestinations[2] = maxPos;
        setLEDState(static_cast<int>(LED::RIGHT_BASE), true);
    } else if (dz < -DEADBAND) {
        motorDestinations[2] = minPos;
        setLEDState(static_cast<int>(LED::RIGHT_BASE), true);
    } else {
        motorDestinations[2] = neutralPos;
        setLEDState(static_cast<int>(LED::RIGHT_BASE), false);
    }

    spdlog::info("dx: {:4.2f}, dy: {:4.2f}, dz: {:4.2f} | Dest: [{:4d}, {:4d}, {:4d}]",
               dx, dy, dz,
               motorDestinations[0], motorDestinations[1], motorDestinations[2]);
}

// Move motors to target positions
bool SystemController::moveMotorPositions() {
    if (motors.empty()) {
        return false;
    }
    
    try {
        // Use the group sync write
        for (int j = 0; j < MOTOR_CNT && j < static_cast<int>(motors.size()); j++) {
            if (!motors[j].setMotorDestination(groupSyncWrite, motorDestinations[j])) {
                spdlog::error("Failed to set motor {} destination", j);
                return false;
            }
        }
        
        // Execute the group sync write
        int dxl_comm_result = groupSyncWrite->txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("Group sync write failed: {}", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        
        groupSyncWrite->clearParam();
        return true;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during motor movement: {}", e.what());
        return false;
    }
}

// Read motor positions to verify they reached target
bool SystemController::readMotorPositions() {
    if (motors.empty()) {
        return false;
    }
    
    try {
        bool allReached = true;
        
        for (int j = 0; j < MOTOR_CNT && j < static_cast<int>(motors.size()); j++) {
            motors[j].checkAndGetPresentPosition(groupSyncRead);
            if (!motors[j].checkIfAtGoalPosition(motorDestinations[j])) {
                allReached = false;
            }

            spdlog::debug("Motor {}: Current: {}, Target: {}, Diff: {}", 
                         j, currentPos, targetPos, std::abs(static_cast<int>(currentPos) - targetPos));
        }
        
        return allReached;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during motor position reading: {}", e.what());
        return false;
    }
}