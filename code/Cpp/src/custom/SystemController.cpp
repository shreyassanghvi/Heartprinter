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

        sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());

        if (!initializeHardware()) {
            handleError(SystemException(SystemError::MOTOR_INIT_FAILED,
                                      "Failed to initialize hardware"));
            return false;
        }
        currentState = States::CALIBRATION;
        initialized = true;

        sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());
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
        analogConf.minVoltage = -10;
        analogConf.maxVoltage = 10;
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
                           spdlog::info("DAQ Channel {} Average Value {}", ch, channelAverages[ch]);
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

        spdlog::info("Initializing load cell.");
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
    int caliberationDestination[] = {neutralPos};
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

            // Set the velocity profile
            if (motors.back().setVelocityProfile(packetHandler, portHandler, DXL_VELOCITY_PROFILE_VALUE) != 0) {
                spdlog::error("Failed to set velocity profile for motor {}", i);
                return false;
            }
        }

        // for (int calCycle = 0; calCycle< 1; calCycle++) {
        //     groupSyncWrite.clearParam();
        //     for (int i = 0; i < MOTOR_CNT; i++) {
        //         motorDestinations[i] = caliberationDestination[calCycle];
        //         if (!motors[i].setMotorDestination(&groupSyncWrite, motorDestinations[i])) {
        //             spdlog::error("Failed to set to {} position for motor {}", motorDestinations[i], i);
        //             return false;
        //         }
        //     }
        //
        //     int dxl_comm_result = groupSyncWrite.txPacket();
        //     if (dxl_comm_result != COMM_SUCCESS) {
        //         spdlog::error("Failed to execute movement to {} motor position: {}", caliberationDestination[calCycle],  packetHandler->getTxRxResult(dxl_comm_result));
        //         return false;
        //     }
        //     while(!this->readMotorPositions()) {
        //         ;
        //     }
        // }
        // groupSyncWrite.clearParam();

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
    // Go through calibration
    currentState = processStateTransition(currentState);

    if (currentState != States::READY) {
        spdlog::error("System not ready - current state: {}", 
                     static_cast<int>(currentState));
        return;
    }

    // Should get us from READY to RUNNING
    currentState = processStateTransition(currentState);
    try {
        spdlog::info("Starting main control loop...");

        // Main control loop - using StateController pattern
        // Only should go into this when RUNNING or MOVING.
        while ((currentState == States::RUNNING || currentState == States::MOVING)) {
            // Process state transitions
            States newState = processStateTransition(currentState);
            if (newState != currentState) {
                std::string state_str = currentStateToString();
                currentState = newState;
                spdlog::info("{} -> {}", state_str, currentStateToString());
            }
            sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());

            // Break if we're no longer in a valid running state
            if (currentState != States::RUNNING && currentState != States::MOVING) {
                break;
            }
            
            // Read tracking data
            try {
                validateProbes();
                GetAsynchronousRecord(3, &currentPosition, sizeof(currentPosition));
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
                    setMotorDestinationsForTarget(desiredPosition);
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
        
        spdlog::info("Control loop completed");
        
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
            return States::CALIBRATION;

        case States::CALIBRATION:
            // Check if all 4 probes are returning values
            if (!validateProbes()) {
                spdlog::error("Missing probe detected, going into calibration routine");

                // If calibration is not yet complete, perform it
                spdlog::info("Starting calibration routine...");
                if (!performCalibration()) {
                    spdlog::error("Calibration routine failed");
                    return States::ERR;
                }
            }

            // Actually set the current position of the moving base.
            GetAsynchronousRecord(3, &currentPosition, sizeof(currentPosition));
            sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());

            spdlog::info("Waiting for DAQ values to become available");
            while (!g_systemControllerInstance->daqDataAvailable) {
                    ;
            }

            while (!performSafetyCheck(currentPosition)) {
                if (!adjustTensionBasedOnLoadCells()) {
                    spdlog::error("Failed to adjust tension - entering error state");
                    return States::ERR;
                }
				while (!readMotorPositions()) {
					;
				}
            }

            // Calculate centroid of the three base positions
            calculateCentroid();
            desiredPosition = centroidPosition;

            MotorCommand sharedCmd;
            sharedCmd.execute = false;
            sharedCmd.target_x = desiredPosition.x;
            sharedCmd.target_y = desiredPosition.y;
            sharedCmd.target_z = desiredPosition.z;
            sharedMemory->writeMotorCommand(sharedCmd);

            // Move to centroid position
            setMotorDestinationsForTarget(desiredPosition);
            spdlog::info("Motor Destinations: {}, {}, {}", motorDestinations[0], motorDestinations[1], motorDestinations[2]);

            spdlog::info("Moving to centroid position...");
            if (!moveMotorPositions()) {
                spdlog::info("Failed to move motors.");
                return States::ERR;
            }
            return States::READY;

        case States::READY:
            return States::RUNNING;

        case States::RUNNING:
            // Check if motors are moving
            if (!readMotorPositions()) {
                return States::MOVING;
            }
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

            return States::RUNNING;

        case States::MOVING:
            // Check if motors have reached their targets
            if (readMotorPositions()) {
                spdlog::debug("Motors reached target positions");
                // We need to check to see whether the trackstar current position actually matches that of the desired position

                GetAsynchronousRecord(3, &currentPosition, sizeof(currentPosition));

                // Calculate vector from current to desired position
                double dx = desiredPosition.x - currentPosition.x;
                double dy = desiredPosition.y - currentPosition.y;
                double dz = desiredPosition.z - currentPosition.z;
                spdlog::info("Current position: ({:.3f}, {:.3f}, {:.3f})", currentPosition.x, currentPosition.y, currentPosition.z);
                spdlog::info("Desired position: ({:.3f}, {:.3f}, {:.3f})", desiredPosition.x, desiredPosition.y, desiredPosition.z);
                spdlog::info("Difference vector (dx, dy, dz): ({:.3f}, {:.3f}, {:.3f})", dx, dy, dz);

                double posError = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
                spdlog::info("Vector magnitude: {:.3f}", posError);

                // If we're somehow very close to the desiredPosition (probably when we're near the bases) go to RUNNING
                if (posError < 1.5) {
                    return States::RUNNING;
                }

                // Calculate plane
                double a1 = staticBasePositions[1].x - staticBasePositions[0].x;
                double b1 = staticBasePositions[1].y - staticBasePositions[0].y;
                double c1 = staticBasePositions[1].z - staticBasePositions[0].z;
                double a2 = staticBasePositions[2].x - staticBasePositions[0].x;
                double b2 = staticBasePositions[2].y - staticBasePositions[0].y;
                double c2 = staticBasePositions[2].z - staticBasePositions[0].z;

                double a = b1 * c2 - b2 * c1;
                double b = a2 * c1 - a1 * c2;
                double c = a1 * b2 - b1 * a2;
                double normalMagnitude = sqrt(pow(a, 2) + pow(b, 2) + pow(c, 2));
                spdlog::info("Plane normal (a, b, c): ({:.3f}, {:.3f}, {:.3f}), magnitude: {:.3f}", a, b, c, normalMagnitude);

                // Calculate angle between vector and plane normal
                // If vector is perpendicular to plane, it should be parallel to normal (angle near 0 or 180)
                double dotProduct = dx*a + dy*b + dz*c;
                double cosAngle = std::max(-1.0, std::min(1.0, dotProduct / (posError * normalMagnitude)));
                spdlog::info("Dot product: {:.3f}, cosAngle: {:.3f}", dotProduct, cosAngle);

                // angleToNormal is angle between vector and normal
                double angleToNormal = acos(std::abs(cosAngle)) * 180.0 / M_PI;
                spdlog::info("Angle to plane normal: {:.1f} deg", angleToNormal);

                // We want vector perpendicular to plane (parallel to normal), so angleToNormal should be near 0
                // TODO: Make this configurable
                const double angleThreshold = 10.0;
                if (angleToNormal > angleThreshold) {
                    spdlog::warn("Position vector not perpendicular to base plane (angle: {:.1f} deg > {:.1f} deg threshold)",
                                angleToNormal, angleThreshold);
                    setMotorDestinationsForTarget(desiredPosition);
                    if (!moveMotorPositions()) {
                        spdlog::error("Failed to move motors");
                        handleError(SystemException(SystemError::MOTOR_INIT_FAILED, "Motor movement failed"));
                        break;
                    }
                    return States::MOVING;
                }
                return States::RUNNING;
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
                spdlog::warn("Safety: Load cell {} underload detected: {:.3f}V > {:.3f}V",
                           i, channelAverages[i], config.maxLoadVoltage);
                return false;
            }

            if (channelAverages[i] < config.minLoadVoltage) {
                spdlog::warn("Safety: Load cell {} overload detected: {:.3f}V < {:.3f}V",
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

    sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());
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


// Convert current state to string
std::string SystemController::currentStateToString() const {
    switch (currentState) {
        case States::START: return "START";
        case States::INITIALIZING: return "INIT";
        case States::CALIBRATION: return "CALIB";
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
    sharedMemory->writeStatusUpdate(currentPosition, staticBasePositions, currentStateToString());

    // Try to read shared memory command
    if (!sharedMemory->readMotorCommand(cmd)) {
        return false;
    }

    // Check if command should be executed
    if (!cmd.execute) {
        return false;  // Command not ready for execution
    }

    if ((std::abs(cmd.target_x) > config.trackingDeadband) ||
        (std::abs(cmd.target_y) > config.trackingDeadband) ||
        (std::abs(cmd.target_z) > config.trackingDeadband)) {
        spdlog::warn("SHM: Target cannot be reached, outside of DEADBAND.");
        return false;
    }

    // Clear the execute flag and write back to shared memory
    MotorCommand clearedCmd = cmd;
    clearedCmd.execute = false;
    desiredPosition.x = cmd.target_x;
    desiredPosition.y = cmd.target_y;
    desiredPosition.z = cmd.target_z;
    sharedMemory->writeMotorCommand(clearedCmd);

    return true;
}


void SystemController::setMotorDestinationsForTarget(DOUBLE_POSITION_ANGLES_RECORD& targetPos) {
    for (int i = 0; i < MOTOR_CNT; i++) {
        double curr_x = currentPosition.x - staticBasePositions[i].x;
        double curr_y = currentPosition.y - staticBasePositions[i].y;
        double curr_z = currentPosition.z - staticBasePositions[i].z;
        double curr_cable_length = sqrt(pow(curr_x, 2) + pow(curr_y, 2) + pow(curr_z, 2));
        int curr_step_count = motors[i].mmToDynamixelUnits(curr_cable_length);
        spdlog::info("Base {}. Current cable length/step count: {}/{}", i, curr_cable_length, curr_step_count);

        double dx = targetPos.x - staticBasePositions[i].x;
        double dy = targetPos.y - staticBasePositions[i].y;
        double dz = targetPos.z - staticBasePositions[i].z;
        double desired_cable_length = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        int desired_step_count = motors[i].mmToDynamixelUnits(desired_cable_length);
        spdlog::info("Desired cable length/step count: {}/{}", desired_cable_length, desired_step_count);

        int actual_curr_step_count = motors[i].checkAndGetPresentPosition(packetHandler, portHandler, &groupSyncRead);
        spdlog::info("Actual motor step count: {}", actual_curr_step_count);

        // Assume:
        // Actual step count = 0, 0, 0
        // Actual position = 1, 1, 1
        // Desired position = 0, 0, 0
        // We would expect:
        // Desired step count = 1, 1, 1 (to go from 1, 1, 1 to 0, 0, 0)

        // Actual position = 0, 0, 0
        // Desired position = 1, 1, 1
        // Expect:
        // Desired step count = -1, -1, -1 ( to go from 0, 0, 0 to 1, 1, 1)
        motorDestinations[i] = actual_curr_step_count + (curr_step_count - desired_step_count);
    }
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

    try {
        groupSyncRead.clearParam();
        bool allReached = true;
        
        for (int j = 0; j < MOTOR_CNT && j < static_cast<int>(motors.size()); j++) {
            uint32_t motor_j_current_pos = motors[j].checkAndGetPresentPosition(packetHandler, portHandler, &groupSyncRead);
            if (!motors[j].checkIfAtGoalPosition(motorDestinations[j])) {
                allReached = false;
            }

            spdlog::debug("Motor {}: Current: {}, Target: {}, Diff: {}", 
                         j, motor_j_current_pos, motorDestinations[j], static_cast<int>(motor_j_current_pos) - motorDestinations[j]);
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

    // Do full adjustment at 0.075V error
    const double Kp = config.tensionAdjustmentSteps / 0.075;
    const int minAdjustment = 10;
    const int maxAdjustment = config.tensionAdjustmentSteps * 3;
    const double midVoltage = (config.minLoadVoltage + config.maxLoadVoltage) / 2;

    for (int i = 0; i < 3; i++) {
        double error = std::abs(midVoltage - channelAverages[i]);
        int adjustment = std::max(minAdjustment, std::min(static_cast<int>(Kp * error), maxAdjustment));
        // This logic is a bit confusing because larger negative values are larger loads.
        // Check if load is too high (overload)
        // Too high means that it is lower than our minimum voltage
        if (channelAverages[i] < config.minLoadVoltage) {
            // Relieve tension by moving motor to decrease cable tension (negative direction)
            adjustment = -adjustment;
            spdlog::info("Motor {} overload ({:.3f}V < {:.3f}V, error={:.3f}V) - relieving tension by {} steps (P-term)",
                       i, channelAverages[i], config.minLoadVoltage, error, -adjustment);
            setLEDState(i, LED_OFF);
        }
        // Check if load is too low (potential sensor issue or slack cable)
        // Too low means that it is higher than our max voltage
        else if (channelAverages[i] > config.maxLoadVoltage) {
            spdlog::info("Motor {} underload ({:.3f}V > {:.3f}V, error={:.3f}V) - increasing tension by {} steps (P-term)",
                       i, channelAverages[i], config.maxLoadVoltage, error, adjustment);
            setLEDState(i, LED_OFF);
        }
        else {
            adjustment = 0;
            setLEDState(i, LED_ON);
        }

        // Apply adjustment if needed
        if (adjustment != 0) {
            int motor_i_current_pos = motors[i].checkAndGetPresentPosition(packetHandler, portHandler, &groupSyncRead);
            motorDestinations[i] = motor_i_current_pos + adjustment;
            adjustmentMade = true;

            spdlog::info("Motor {} adjusted: current position = {}, new destination = {} (P-gain={:.1f})",
                       i, motor_i_current_pos, motorDestinations[i], Kp);
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

// Calibration Functions

// Validate that all 4 probes are returning valid data
// Also stores the static base positions (sensors 0-2) into staticBasePositions array
bool SystemController::validateProbes() {
    spdlog::info("Validating tracking probes...");

    int sensorCount = getConnectedSensors();
    spdlog::info("Found {} connected sensors", sensorCount);

    bool all_connected = true;

    if (sensorCount < 4) {
        spdlog::error("Insufficient sensors detected. Expected 4 sensors (1 moving + 3 static bases), found {}", sensorCount);
        all_connected = false;
    }

    // Try to read from all 4 sensors
    for (short sensorID = 0; sensorID < 4; sensorID++) {
        DOUBLE_POSITION_ANGLES_RECORD record;
        record.x = -100;
        record.y = -100;
        record.z = -100;

        int errorCode = GetAsynchronousRecord(sensorID, &record, sizeof(record));
        if (errorCode != BIRD_ERROR_SUCCESS) {
            spdlog::error("Failed to read from sensor {}", sensorID);
            all_connected = false;
            continue;
        }

        unsigned int status = GetSensorStatus(sensorID);
        if (status != VALID_STATUS) {
            spdlog::error("Sensor {} status invalid: 0x{:X}", sensorID, status);
            all_connected = false;
            continue;
        }

        spdlog::info("Sensor {} validated: ({:.2f}, {:.2f}, {:.2f})",
                    sensorID, record.x, record.y, record.z);

        // Store static base positions (sensors 0, 1, 2)
        if (sensorID < 3) {
            staticBasePositions[sensorID] = record;
            spdlog::info("Static base {} position stored: ({:.2f}, {:.2f}, {:.2f})",
                        sensorID, record.x, record.y, record.z);
        }
    }

    spdlog::info("All 4 probes connected?: {}", all_connected);
    spdlog::info("Static base positions recorded for connected sensors.");
    return all_connected;
}

// Detect if motors have reached the base by checking position stability
// Returns true if the motor has stalled (not moving AND not at commanded position)
bool SystemController::detectBaseArrival(int motorIndex) {
    if (motorIndex < 0 || motorIndex >= MOTOR_CNT) {
        spdlog::error("Invalid motor index for base arrival detection: {}", motorIndex);
        return false;
    }

    int previousPosition = -1;
    int stableReadCount = 0;

    spdlog::debug("Waiting for motor {} to stall at base...", motorIndex);

    while (stableReadCount < config.calibrationStabilityReads) {
        // Read current motor position
        int currentPos = motors[motorIndex].checkAndGetPresentPosition(packetHandler, portHandler, &groupSyncRead);

        // Check if we've reached the commanded position
        if (motors[motorIndex].checkIfAtGoalPosition(motorDestinations[motorIndex])) {
            spdlog::debug("Motor {} reached commanded position {} - not stalled",
                         motorIndex, motorDestinations[motorIndex]);
            return false;  // Motor reached target, not blocked by base
        }

        // Check if position has changed significantly from previous read
        if (previousPosition != -1) {
            int posDiff = std::abs(currentPos - previousPosition);
            if (posDiff > config.calibrationPositionTolerance) {
                // Motor is still moving, reset stable count
                stableReadCount = 0;
                spdlog::debug("Motor {} still moving: pos={}, diff={}", motorIndex, currentPos, posDiff);
            } else {
                // Motor position hasn't changed - increment stable count
                stableReadCount++;
                spdlog::debug("Motor {} stable read {}/{}: pos={}",
                             motorIndex, stableReadCount, config.calibrationStabilityReads, currentPos);
            }
        }

        previousPosition = currentPos;

        // Small delay between reads
        Sleep(100);
    }

    spdlog::info("Motor {} stalled at base (stable for {} reads at position {})",
                motorIndex, config.calibrationStabilityReads, previousPosition);
    return true;
}

// Move to a specific static base by retracting one motor and extending the others
bool SystemController::moveToBase(int baseIndex) {
    if (baseIndex < 0 || baseIndex >= 3) {
        spdlog::error("Invalid base index: {}. Must be 0-2", baseIndex);
        return false;
    }

    spdlog::info("Moving to static base {}...", baseIndex);

    bool baseReached = false;
    while (!baseReached) {
        // Set motor destinations
        // Retract the motor associated with this base by N steps, extend the other two motors by N steps
        groupSyncWrite.clearParam();
        for (int i = 0; i < MOTOR_CNT; i++) {
            motorDestinations[i] = i == baseIndex ? motorDestinations[i] + config.calibrationMovementSteps : motorDestinations[i] - config.calibrationMovementSteps;

            if (!motors[i].setMotorDestination(&groupSyncWrite, motorDestinations[i])) {
                spdlog::error("Failed to set destination for motor {} when moving to base {}", i, baseIndex);
                return false;
            }

            spdlog::info("Motor {} destination set to {} (base {} calibration)", i, motorDestinations[i], baseIndex);
        }

        // Execute movement
        int dxl_comm_result = groupSyncWrite.txPacket();
        if (dxl_comm_result != COMM_SUCCESS) {
            spdlog::error("Failed to execute movement to base {}: {}",
                         baseIndex, packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        }

        // Wait for arrival at base by checking if the retracting motor has stalled
        if (detectBaseArrival(baseIndex)) {
            spdlog::info("Base {} reached - motor {} stalled", baseIndex, baseIndex);
            baseReached = true;
        } else {
            spdlog::debug("Motor {} not yet stalled, continuing movement to base {}", baseIndex, baseIndex);
        }
    }

    // Read and store the tracking position at this base
    try {
        staticBasePositions[baseIndex] = readATI(3); // Read from moving base sensor
        spdlog::info("Base {} position recorded: ({:.2f}, {:.2f}, {:.2f})",
                    baseIndex,
                    staticBasePositions[baseIndex].x,
                    staticBasePositions[baseIndex].y,
                    staticBasePositions[baseIndex].z);
    } catch (const std::exception& e) {
        spdlog::error("Failed to read tracking position at base {}: {}", baseIndex, e.what());
        return false;
    }

    return true;
}

// Calculate the centroid of the three static base positions
void SystemController::calculateCentroid() {
    spdlog::info("Calculating centroid of static base positions...");

    centroidPosition.x = (staticBasePositions[0].x + staticBasePositions[1].x + staticBasePositions[2].x) / 3.0;
    centroidPosition.y = (staticBasePositions[0].y + staticBasePositions[1].y + staticBasePositions[2].y) / 3.0;
    centroidPosition.z = (staticBasePositions[0].z + staticBasePositions[1].z + staticBasePositions[2].z) / 3.0;

    spdlog::info("Centroid position: ({:.2f}, {:.2f}, {:.2f})",
                centroidPosition.x, centroidPosition.y, centroidPosition.z);
}

// Perform full calibration routine
bool SystemController::performCalibration() {
    spdlog::info("Starting Calibration Routine");

    // Record initial position (first base position is starting position)
    try {
        staticBasePositions[1] = readATI(3);
        spdlog::info("Initial position (base 0) recorded: ({:.2f}, {:.2f}, {:.2f})",
                    staticBasePositions[0].x,
                    staticBasePositions[0].y,
                    staticBasePositions[0].z);
    } catch (const std::exception& e) {
        spdlog::error("Failed to read initial position: {}", e.what());
        return false;
    }

    // Move to the other two static bases and record their positions
    for (int baseIndex = 0; baseIndex < 3; baseIndex++) {
        if (baseIndex == 1) {
            continue;
        }

        if (!moveToBase(baseIndex)) {
            spdlog::error("Failed to move to base {}", baseIndex);
            return false;
        }
    }

    groupSyncWrite.clearParam();

    spdlog::info("Calibration Complete");

    return true;
}
