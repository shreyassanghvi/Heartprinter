//
// SystemController.cpp - Implementation of the Simplified Heart Printer System Controller
//

#include "../../include/custom/SystemController.h"
#include "../../include/custom/Motor.h"
#include "../../include/custom/CTrackstar.h"
#include "../../include/custom/NIDAQ.h"
#include "../../include/custom/SharedMemoryManager.h"
#include "../../include/Trackstar/ATC3DG.h"

#include <spdlog/spdlog.h>
#include <chrono>
#include <thread>
#include <cmath>
#include <functional>

#ifdef _WIN32
#include <windows.h>
#endif

// Motor system constants
#define MOTOR_CNT 3

// Static pointer for DAQ callback access
static SystemController* g_systemControllerInstance = nullptr;

// Constructor
SystemController::SystemController(const SystemConfig& config)
    : config(config), currentState(States::START), 
    portHandler(dynamixel::PortHandler::getPortHandler(config.deviceName.c_str())),
    packetHandler(dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION)), 
    groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION),
    groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) {
    spdlog::info("SystemController created with device: {}", config.deviceName);

    // Initialize motor control variables
    minPos = DXL_MINIMUM_POSITION_VALUE;
    maxPos = static_cast<int>(DXL_MAXIMUM_POSITION_VALUE);
    neutralPos = (minPos + maxPos) / 2;
    motorDestinations.resize(MOTOR_CNT, neutralPos);

    // Create shared memory manager
    sharedMemory = std::make_unique<SharedMemoryManager>();

    // Set global instance for DAQ callback
    g_systemControllerInstance = this;
}

// Destructor - RAII cleanup
SystemController::~SystemController() {
    try {
        // Clear global instance pointer
        if (g_systemControllerInstance == this) {
            g_systemControllerInstance = nullptr;
        }

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
    // Initialize DAQ system
    if (!initializeDAQ()) {
        throw SystemException(SystemError::DAQ_INIT_FAILED,
                            "Failed to initialize DAQ system");
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
        analogConf.minVoltage = -3;
        analogConf.maxVoltage = 2;
        analogConf.sampleRate = 1000.0;
        analogConf.samplesPerCallback = 1000;

        // Create DAQ system
        daqSystem = std::make_unique<DAQSystem>();
        *daqSystem = initDAQSystem(digitalConf, analogConf,
           [](double* data, uInt32 numSamples) {
               // Static callback wrapper - accesses SystemController via global pointer
               try {
                   if (!data || numSamples == 0) {
                       spdlog::warn("Invalid data or sample count in DataHandler!");
                       return;
                   }

                   spdlog::info("DAQ - Total data points: {}", sizeof(data));

                   // Assuming that the data is interleaved [channel1, channel2, channel3, ... channel1, channel2, channel3]
                   const uInt32 numChannels = 3;  // ai0:2 = 3 channels

                   // Calculate average for each channel
                   double channelAverages[numChannels] = {0.0, 0.0, 0.0};

                   for (uInt32 i = 0; i < numSamples; i++) {
                       for (uInt32 ch = 0; ch < numChannels; ch++) {
                           channelAverages[ch] += data[i * numChannels + ch] / numSamples;
                       }
                   }

                   // Store in SystemController instance (thread-safe)
                   if (g_systemControllerInstance) {
                       std::lock_guard<std::mutex> lock(g_systemControllerInstance->daqDataMutex);
                       for (uInt32 ch = 0; ch < numChannels; ch++) {
                           g_systemControllerInstance->daqChannelAverages[ch] = channelAverages[ch];
                       }
                       g_systemControllerInstance->daqDataAvailable = true;
                   }

                   spdlog::info("DAQ - Samples/Ch: {} - Avg[Ch0]: {:8.5f}, Avg[Ch1]: {:8.5f}, Avg[Ch2]: {:8.5f}",
                       numSamples,
                       channelAverages[0], channelAverages[1], channelAverages[2]);
               } catch (std::exception &e) {
                   spdlog::error("Error: {}", e.what());
               }
           },
           [](const char* errorMessage) {
               spdlog::error("DAQ Error: {}", errorMessage);
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
        if (!setAllLEDs(false)) {
            spdlog::error("Failed to set all LEDs off");
            return false;
        }

        // TODO: maybe move into a separate function
        spdlog::warn("TODO: Initializing load cell...");
        if (DAQStart(daqSystem->analogHandle) != 0){
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
    int caliberationDestination[] = {maxPos, minPos, neutralPos};
    try {
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
            motors.emplace_back(i);
            
            // Initialize each motor (logic from initMotors function in Init.cpp)
            // Set motor operation mode to extended position control
            if (motors.back().setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE) != EXTENDED_POSITION_CONTROL_MODE) {
                spdlog::error("Failed to set operation mode for motor {}", i);
                return false;
            }
            
            // Enable motor torque
            if (motors.back().enableTorque(packetHandler, portHandler) != 0) {
                spdlog::error("Failed to enable torque for motor {}", i);
                return false;
            }
        }
        for (int calCycle = 0; calCycle< 3; calCycle++) {
            groupSyncWrite.clearParam();
            for (int i = 0; i < MOTOR_CNT; i++) {
                motorDestinations[i] = caliberationDestination[calCycle];
                if (!motors[i].setMotorDestination(&groupSyncWrite, motorDestinations[i])) {
                    spdlog::error("Failed to set to {} position for motor {}", motorDestinations[i], i);
                    return false;
                }
            }

            int dxl_comm_result = groupSyncWrite.txPacket();
            if (dxl_comm_result != COMM_SUCCESS) {
                spdlog::error("Failed to execute movement to {} motor position: {}", caliberationDestination[calCycle],  packetHandler->getTxRxResult(dxl_comm_result));
                return false;
            }
            while(!this->readMotorPositions()) {
                ;
            }
        }
        groupSyncWrite.clearParam();

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
        tracker = std::make_unique<CSystem>();
        
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
                std::string state_str = currentStateToString();
                currentState = newState;
                spdlog::info("{} -> {}", state_str, currentStateToString());
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
                spdlog::warn("Safety check failed - tension outside of normal bounds.");

                // Attempt to adjust motor positions to correct tension
                if (!adjustTensionBasedOnLoadCells()) {
                    spdlog::error("Failed to adjust tension - entering error state");
                    return States::ERR;
                }

                // After adjustment, transition to MOVING state to let motors reach new positions
                return States::MOVING;
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
                spdlog::warn("Safety check failed - tension outside of normal bounds.");
                // TODO: Adjust the motor positions? We need to relieve or increase tension.
                //  Should this be a different function? executeTensionAdjustment?
            }

            return States::MOVING;

        case States::ERR:
            return States::CLEANUP;

        case States::CLEANUP:
            return States::END;

        case States::END:
            return States::END;

        default:
            spdlog::error("Unknown state: {}", static_cast<int>(current));
            return States::ERR;
    }
}

// Safety checks
bool SystemController::performSafetyCheck(const DOUBLE_POSITION_ANGLES_RECORD& targetPos) const {
    if (!config.enableSafetyChecks) {
        return true;
    }

    // Check DAQ channel averages (load cells)
    double channelAverages[3];
    if (getDAQChannelAverages(channelAverages)) {
        for (int i = 0; i < 3; i++) {
            if (channelAverages[i] > config.maxLoadVoltage) {
                spdlog::warn("Safety: Load cell {} overload detected: {:.3f}V > {:.3f}V",
                           i, channelAverages[i], config.maxLoadVoltage);
                return false;
            }

            if (channelAverages[i] < config.minLoadVoltage) {
                spdlog::warn("Safety: Load cell {} may be disconnected: {:.3f}V < {:.3f}V",
                           i, channelAverages[i], config.minLoadVoltage);
                return false;
            }
        }

        spdlog::debug("Safety: Load cells OK - Ch0:{:.3f}V Ch1:{:.3f}V Ch2:{:.3f}V",
                    channelAverages[0], channelAverages[1], channelAverages[2]);
    } else {
        spdlog::debug("Safety: DAQ data not yet available, skipping load cell check");
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

// Get DAQ channel averages (thread-safe)
bool SystemController::getDAQChannelAverages(double averages[3]) const {
    if (!daqDataAvailable) {
        return false;  // No data available yet
    }

    try {
        std::lock_guard<std::mutex> lock(daqDataMutex);
        for (int i = 0; i < 3; i++) {
            averages[i] = daqChannelAverages[i];
        }
        return true;
    } catch (const std::exception& e) {
        spdlog::error("Exception reading DAQ channel averages: {}", e.what());
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
    currentState = States::ERR;
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
        case States::ERR: return "ERR";
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
    if (!sharedMemory->readMotorCommand(cmd)) {
        return false;
    }

    // Check if command should be executed
    if (!cmd.execute) {
        return false;  // Command not ready for execution
    }

    // Clear the execute flag and write back to shared memory
    MotorCommand clearedCmd = cmd;
    clearedCmd.execute = false;
    sharedMemory->writeMotorCommand(clearedCmd);

    return true;
}

// Calculate motor positions from shared memory command
void SystemController::calculateMotorPositionsFromCommand(const MotorCommand& cmd) {
    if ((std::abs(cmd.target_x) > config.trackingDeadband) ||
        (std::abs(cmd.target_y) > config.trackingDeadband) ||
        (std::abs(cmd.target_z) > config.trackingDeadband)) {
        spdlog::warn("SHM: Target cannot be reached, outside of DEADBAND.");
    }

    // Convert mm to Dynamixel units using motor's conversion function
    if (!motors.empty()) {
        motorDestinations[0] = motors[0].mmToDynamixelUnits(cmd.target_x);
        motorDestinations[1] = motors[1].mmToDynamixelUnits(cmd.target_y);
        motorDestinations[2] = motors[2].mmToDynamixelUnits(cmd.target_z);

        spdlog::info("SHM: Target ({:.2f}, {:.2f}, {:.2f}) -> Dest: [{:4d}, {:4d}, {:4d}]",
                    cmd.target_x, cmd.target_y, cmd.target_z,
                    motorDestinations[0], motorDestinations[1], motorDestinations[2]);
    }
}

// Calculate motor positions from tracking data
void SystemController::calculateMotorPositionsFromTracking() {
    double dx = currentPosition.x;
    double dy = currentPosition.y;
    double dz = currentPosition.z;

    // X axis (motor 0): normal logic
    if (dx > config.trackingDeadband) {
        motorDestinations[0] = maxPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), true);
    } else if (dx < -config.trackingDeadband) {
        motorDestinations[0] = minPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), true);
    } else {
        motorDestinations[0] = neutralPos;
        setLEDState(static_cast<int>(LED::LEFT_BASE), false);
    }

    // Y axis (motor 1): inverted logic
    if (dy > config.trackingDeadband) {
        motorDestinations[1] = minPos;  // inverted
        setLEDState(static_cast<int>(LED::CENTER_BASE), true);
    } else if (dy < -config.trackingDeadband) {
        motorDestinations[1] = maxPos;  // inverted
        setLEDState(static_cast<int>(LED::CENTER_BASE), true);
    } else {
        motorDestinations[1] = neutralPos;
        setLEDState(static_cast<int>(LED::CENTER_BASE), false);
    }

    // Z axis (motor 2): normal logic
    if (dz > config.trackingDeadband) {
        motorDestinations[2] = maxPos;
        setLEDState(static_cast<int>(LED::RIGHT_BASE), true);
    } else if (dz < -config.trackingDeadband) {
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

            if (!motors[j].setMotorDestination(&groupSyncWrite, motorDestinations[j])) {
                spdlog::error("Failed to set motor {} destination", j);
                return false;
            }
        }
        
        // Execute the group sync write
        int dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("Group sync write failed: {}", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }
        
        groupSyncWrite.clearParam();
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
    int dxl_comm_result = 0;
    uint8_t dxl_error = 0;
    try {
        groupSyncRead.clearParam();
        // for (int i = 0; i < MOTOR_CNT; i++) {
        //     groupSyncRead.addParam(motors[i].getMotorID());
        // }
        // dxl_comm_result = groupSyncRead.txRxPacket();
        // if (dxl_comm_result != COMM_SUCCESS) {
        //     spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
        //     for (int i = 0; i < MOTOR_CNT; i++) {
        //         if (groupSyncRead.getError(motors[i].getMotorID(), &dxl_error)) {
        //             spdlog::error("[ID:{:3d}] {}\n", motors[i].getMotorID(),
        //                 packetHandler->getRxPacketError(dxl_error));
        //         }
        //     }
        //     return false;
        // }

        bool allReached = true;
        
        for (int j = 0; j < MOTOR_CNT && j < static_cast<int>(motors.size()); j++) {
            uint32_t motor_j_current_pos = motors[j].checkAndGetPresentPosition(&groupSyncRead);
            if (!motors[j].checkIfAtGoalPosition(motorDestinations[j])) {
                allReached = false;
            }

            spdlog::debug("Motor {}: Current: {}, Target: {}, Diff: {}", 
                         j, motor_j_current_pos, motorDestinations[j], std::abs(static_cast<int>(motor_j_current_pos) - motorDestinations[j]));
        }
        
        return allReached;
        
    } catch (const std::exception& e) {
        spdlog::error("Exception during motor position reading: {}", e.what());
        return false;
    }
}

// Adjust motor positions based on load cell readings to maintain safe tension
bool SystemController::adjustTensionBasedOnLoadCells() {
    double channelAverages[3];
    if (!getDAQChannelAverages(channelAverages)) {
        spdlog::warn("Cannot adjust tension - DAQ data not available");
        return false;
    }

    bool adjustmentMade = false;

    for (int i = 0; i < 3; i++) {
        int adjustment = 0;

        // Check if load is too high (overload)
        if (channelAverages[i] > config.maxLoadVoltage) {
            // Relieve tension by moving motor to decrease cable tension
            // For a cable system, moving toward neutralPos typically relieves tension
            adjustment = -config.tensionAdjustmentSteps;
            spdlog::info("Motor {} overload ({:.3f}V > {:.3f}V) - relieving tension by {} steps",
                       i, channelAverages[i], config.maxLoadVoltage, -adjustment);
        }
        // Check if load is too low (potential sensor issue or slack cable)
        else if (channelAverages[i] < config.minLoadVoltage) {
            // Increase tension by moving motor to tighten cable
            adjustment = config.tensionAdjustmentSteps;
            spdlog::info("Motor {} underload ({:.3f}V < {:.3f}V) - increasing tension by {} steps",
                       i, channelAverages[i], config.minLoadVoltage, adjustment);
        }

        // Apply adjustment if needed
        if (adjustment != 0) {
            // Motor 1 (Y-axis) has inverted logic - flip the adjustment direction
            int finalAdjustment = (i == 1) ? -adjustment : adjustment;

            int newDestination = std::max(minPos, std::min(maxPos, motorDestinations[i] + finalAdjustment));

            motorDestinations[i] = newDestination;
            adjustmentMade = true;

            spdlog::info("Motor {} adjusted: new destination = {} (inverted: {})",
                       i, newDestination, (i == 1 ? "yes" : "no"));
        }
    }

    // If adjustments were made, send the new motor commands
    if (adjustmentMade) {
        if (!moveMotorPositions()) {
            spdlog::error("Failed to apply tension adjustments");
            return false;
        }
        spdlog::info("Tension adjustment applied successfully");
    } else {
        spdlog::debug("No tension adjustment needed - all loads within acceptable range");
    }

    return true;
}
