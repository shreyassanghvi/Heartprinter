//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/Init.h"

#include <iomanip>
#include <unistd.h>

#include "../../include/Trackstar/ATC3DG.h"

#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <filesystem>

#include <chrono>
#include <sstream>
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#endif

namespace fs = std::filesystem;

// Shared memory structure for motor commands
struct MotorCommand {
    double target_x;
    double target_y; 
    double target_z;
    int motor_positions[3];
    bool use_positions;  // If true, use motor_positions directly; if false, calculate from target_xyz
    bool enabled;
    char padding[7]; // Ensure struct size is multiple of 8
};

// Global shared memory variables
#ifdef _WIN32
HANDLE hMapFile = NULL;
#else
int shm_fd = -1;
#endif
MotorCommand* pSharedData = nullptr;
const char* SHM_NAME = "Local\\PyToCPP_Motors";
const size_t SHM_SIZE = sizeof(MotorCommand);
// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
// #define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// #define BAUDRATE                        57600

//user defined #define
#define RECORD_CNT                          1000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control

enum TRACKSTAR_SENSORS_ID {
    LEFT_SENSOR = 0,
    BASE_SENSOR,
    RIGHT_SENSOR,
    MOVING_BASE_SENSOR,
};

STATES state;
// Initialize PortHandler instance
// Set the port path
// Get methods and members of PortHandlerLinux or PortHandlerWindows
dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

// Initialize PacketHandler instance
// Set the protocol version
// Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);

// Initialize GroupSyncWrite instance
dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);
// Initialize Groupsyncread instance for Present Position
dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);


int setupMotorPort() {
    if (portHandler->openPort()) {
        spdlog::info("Succeeded to open the port!");
    } else {
        spdlog::error("Failed to open the port!");
        spdlog::error("Press any key to terminate...");
        _getch();
        return EXIT_FAILURE;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        spdlog::info("Succeeded to change the baudrate!");
    } else {
        spdlog::error("Failed to change the baudrate!");
        spdlog::error("Press any key to terminate...");
        _getch();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int initMotors(Motor motors) {
    uint8_t dxl_error = 0; // Dynamixel error
    motors.setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE);
    // Enable Dynamixel#i Torque
    dxl_error += motors.enableTorque(packetHandler, portHandler);
    if (dxl_error != 0) {
        spdlog::error("Failed to enable torque for motor #{}", motors.getMotorID());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}

void cleanUpAndExit() {
    int errorCode = SaveSystemConfiguration("newfile.ini");
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // Turn off the transmitter before exiting
    // We turn off the transmitter by "selecting" a transmitter with an id of "-1"
    //
    short id = -1;
    errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (errorCode != BIRD_ERROR_SUCCESS) errorHandler(errorCode, __LINE__);
}

void DataHandler(double *data, uInt32 numSamples) {
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

        state = ERR;
    }
}

void ErrorHandlerDAQ(const char *errorMessage) {
    spdlog::error("DAQ Error: {}", errorMessage);
    state = ERR;
}

// Initialize shared memory for reading motor commands
bool initSharedMemory() {
#ifdef _WIN32
    // Windows implementation
    hMapFile = OpenFileMapping(
        FILE_MAP_READ,
        FALSE,
        SHM_NAME);
    
    if (hMapFile == NULL) {
        spdlog::warn("Shared memory not found, motor commands will use trackstar data only");
        return false;
    }
    
    pSharedData = (MotorCommand*)MapViewOfFile(
        hMapFile,
        FILE_MAP_READ,
        0,
        0,
        SHM_SIZE);
    
    if (pSharedData == NULL) {
        spdlog::error("MapViewOfFile failed: {}", GetLastError());
        CloseHandle(hMapFile);
        hMapFile = NULL;
        return false;
    }
#else
    // Linux implementation
    shm_fd = shm_open(SHM_NAME, O_RDONLY, 0666);
    if (shm_fd == -1) {
        spdlog::warn("Shared memory not found, motor commands will use trackstar data only");
        return false;
    }
    
    pSharedData = (MotorCommand*)mmap(0, SHM_SIZE, PROT_READ, MAP_SHARED, shm_fd, 0);
    if (pSharedData == MAP_FAILED) {
        spdlog::error("mmap failed");
        close(shm_fd);
        shm_fd = -1;
        return false;
    }
#endif
    
    spdlog::info("Shared memory initialized successfully");
    return true;
}

// Cleanup shared memory
void cleanupSharedMemory() {
    if (pSharedData != nullptr) {
#ifdef _WIN32
        UnmapViewOfFile(pSharedData);
        if (hMapFile != NULL) {
            CloseHandle(hMapFile);
            hMapFile = NULL;
        }
#else
        munmap(pSharedData, SHM_SIZE);
        if (shm_fd != -1) {
            close(shm_fd);
            shm_fd = -1;
        }
#endif
        pSharedData = nullptr;
        spdlog::info("Shared memory cleaned up");
    }
}

// Read motor command from shared memory
bool readMotorCommand(MotorCommand& cmd) {
    if (pSharedData == nullptr) {
        return false;
    }
    
    // Copy data atomically
    memcpy(&cmd, pSharedData, sizeof(MotorCommand));
    return true;
}

std::shared_ptr<spdlog::logger> create_dated_logger(bool make_default) {
    std::string log_dir = "../../logs/cpp";
    std::filesystem::create_directories("log_dir");

    // Get current time
    auto now = std::chrono::system_clock::now();
    std::time_t tt = std::chrono::system_clock::to_time_t(now);

    std::tm tm;
#ifdef _WIN32
    localtime_s(&tm, &tt);
#else
    localtime_r(&tt, &tm);
#endif

    std::ostringstream oss;
    oss << log_dir<<"/log_"
            << std::put_time(&tm, "%Y%m%d_%H%M%S")
            << ".txt";

    std::string filename = oss.str();
    try {
        // Create sinks: file sink and color console sink
        auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(filename, true);
        auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

        std::vector<spdlog::sink_ptr> sinks{file_sink, console_sink};

        // Create logger with both sinks
        auto logger = std::make_shared<spdlog::logger>("multi_sink", sinks.begin(), sinks.end());

        if (make_default) {
            register_logger(logger);
            set_default_logger(logger);
        }
        spdlog::set_level(spdlog::level::debug);
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$]:\t%v"); // Coloring enabled with %^
        spdlog::info("Logger initialized with both file and console sinks");
        return logger;
    } catch (const std::exception &e) {
        spdlog::error("Failed to create logger: {}", e.what());
        return nullptr;
    }
}

//Init commit for Control loop code
int main(int argc, char *argv[]) {
    state = START;
    // Motor motors[MOTOR_CNT];
    std::vector<Motor> vMotors;
    int dxl_comm_result; // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, static_cast<int>(DXL_MAXIMUM_POSITION_VALUE * 1.5)};
    uint8_t dxl_error; // Dynamixel error
    long data_count = 0;
    int ERR_FLG = 0;
    int motor_destination[MOTOR_CNT];

    DAQSystem daqSystem;
    DOUBLE_POSITION_ANGLES_RECORD retRecord{};

    //TODO: check and fix state machine
    while (true) {
        switch (state) {
            case START:
                dxl_comm_result = COMM_SUCCESS;
                groupSyncWrite.clearParam();
                state = INIT_DAQ;
            // setLogLevel(LOG_FATAL);
                create_dated_logger(true);
                break;
            case INIT_DAQ: {
                spdlog::info("Initializing the daq...");
                DigitalConfig digiConfig = {"Dev1", "PFI0:2"};
                AnalogConfig analogConfig = {
                    "Dev1", "ai0:2",
                    0.0, 10.0,
                    10000.0,
                    1000
                };
                spdlog::info("DigitalConfig - Device: {}, Channel: {}", digiConfig.device, digiConfig.channel);

                spdlog::info("AnalogConfig - Device: {}, Channel: {}, MinVoltage: {:.2f}, MaxVoltage: {:.2f}, SampleRate: {:.2f}, SamplesPerCallback: {}",
                             analogConfig.device, analogConfig.channel,
                             analogConfig.minVoltage, analogConfig.maxVoltage,
                             analogConfig.sampleRate, analogConfig.samplesPerCallback);

                daqSystem = initDAQSystem(digiConfig, analogConfig, DataHandler, ErrorHandlerDAQ);
                if (!daqSystem.initialized) {
                    spdlog::error("Failed to initialize DAQ system");
                    // printLog(LOG_ERROR, buf);
                    state = ERR;
                    break;
                }
                if (setAllLEDs(daqSystem.digitalTask, LED_ON) != EXIT_SUCCESS) {
                    spdlog::error("Failed to set all LEDs off");
                    // printLog(LOG_ERROR, buf);
                    state = ERR;
                    break;
                }
                sleep(1);
                if (setAllLEDs(daqSystem.digitalTask, LED_OFF) != EXIT_SUCCESS) {
                    spdlog::error("Failed to set all LEDs off");
                    // printLog(LOG_ERROR, buf);
                    state = ERR;
                    break;
                }
                state = INIT_TRACKSTAR; //INIT_MOTORS;
                break;
            }
            case INIT_MOTORS:
                spdlog::info("Initializing motors...");
                if (setupMotorPort() != EXIT_SUCCESS) {
                    state = ERR;
                    ERR_FLG = 1;
                    break;
                }
                for (int i = 0; i < MOTOR_CNT; i++) {
                    motor_destination[i] = 0;
                    vMotors.emplace_back(i);
                    if (initMotors(vMotors.back()) != EXIT_SUCCESS) {
                        state = ERR;
                        ERR_FLG = 1;
                        break;
                    }
                    if (vMotors.back().setMotorDestination(&groupSyncWrite, DXL_MINIMUM_POSITION_VALUE) != true) {
                        state = ERR;
                        ERR_FLG = 1;
                        break;
                    }
                }
                dxl_comm_result = groupSyncWrite.txPacket();
                if (dxl_comm_result != COMM_SUCCESS) {
                    spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
                    state = ERR;
                }
                groupSyncWrite.clearParam();
                if (state == ERR) {
                    break;
                }
                state = INIT_TRACKSTAR;
                break;
            case INIT_TRACKSTAR:
                initTxRx();
                state = INIT_LOADCELL;
                break;

            case INIT_LOADCELL:
                //TODO: Initialize load cell
                spdlog::warn("TODO: Initializing load cell...");
                DAQStart(daqSystem.analogHandle);
                
                // Initialize shared memory for motor commands
                spdlog::info("Initializing shared memory...");
                initSharedMemory(); // Non-critical - continues even if it fails
                
                state = READ_TRACKSTAR;
                break;

            case READ_TRACKSTAR:
                if (data_count == 0) {
                    spdlog::info("Collect {} Data records from Sensors",RECORD_CNT);

                    spdlog::info("Metric mode was selected, position is in mm.");
                }
                if (data_count < RECORD_CNT) {
                    for (auto i = 0; i < getConnectedSensors(); i++) {
                        retRecord = readATI(i);
                        data_count++;
                        std::string sensorName;
                        switch (i) {
                            case LEFT_SENSOR:
                                sensorName = "L";
                                break;
                            case RIGHT_SENSOR:
                                sensorName = "R";
                                break;
                            case BASE_SENSOR:
                                sensorName = "C";
                                break;
                            case MOVING_BASE_SENSOR:
                                sensorName = "M";
                                break;
                            default:
                                sensorName = "Unknown";
                                spdlog::error("Incorrect sensor ID");
                                state = ERR;
                                break;
                        }

                        spdlog::info("TS: [{}] {:8.3f} {:8.3f} {:8.3f} : {:8.3f} {:8.3f} {:8.3f}",
                                     sensorName,
                                     retRecord.x, retRecord.y, retRecord.z,
                                     retRecord.a, retRecord.e, retRecord.r);
                    }
                    if (state != ERR) {
                        state = READ_DAQ;
                    }
                } else {
                    state = END;
                }
                break;
            case READ_DAQ:
                state = SET_MOTOR;
                break;
            case SET_MOTOR: {
                static const double DEADBAND = 3.0;
                static const double REF_X = 0.0;
                static const double REF_Y = 0.0;
                static const double REF_Z = 0.0;
                
                // Try to read shared memory command first
                MotorCommand sharedCmd;
                bool useSharedMemory = readMotorCommand(sharedCmd);
                
                if (useSharedMemory && sharedCmd.enabled) {
                    // Use shared memory commands
                    if (sharedCmd.use_positions) {
                        // Direct motor position control
                        for (int i = 0; i < MOTOR_CNT; i++) {
                            motor_destination[i] = sharedCmd.motor_positions[i];
                        }
                        spdlog::info("SHM: Direct motor positions: [{:4d}, {:4d}, {:4d}]",
                                   motor_destination[0], motor_destination[1], motor_destination[2]);
                    } else {
                        spdlog::info("SHM: Target ({:.2f}, {:.2f}, {:.2f}) -> Dest: [{:4d}, {:4d}, {:4d}]",
                                   sharedCmd.target_x, sharedCmd.target_y, sharedCmd.target_z,
                                   motor_destination[0], motor_destination[1], motor_destination[2]);
                        // Calculate motor positions from target coordinates
                        // If any of the targets are outside of the DEADBAND, we log and set message in write shared buffer
                        //  and skip this movement
                        if ((DEADBAND < sharedCmd.target_x < -DEADBAND) || (DEADBAND < sharedCmd.target_y < -DEADBAND) || (DEADBAND < sharedCmd.target_z < -DEADBAND)) {
                            spdlog::warning("SHM: Target cannot be reached, outside of DEADBAND.")
                        }
                        
                        // If all of the targets are inside of the DEADBAND, we can set the moves
                        motor_destination[0] = Motor.mmToDynamixelUnits(sharedCmd.target_x);
                        motor_destination[1] = Motor.mmToDynamixelUnits(sharedCmd.target_y);
                        motor_destination[2] = Motor.mmToDynamixelUnits(sharedCmd.target_z);
                        spdlog::info("Motors: Motor positions set.")
                    }
                    
                    // Enable motor movement for shared memory commands
                    state = MOVE_MOTORS;
                } else {
                    // Fallback to trackstar-based control (original logic)
                    double dx = retRecord.x - REF_X;
                    double dy = retRecord.y - REF_Y;
                    double dz = retRecord.z - REF_Z;

                    // X axis (motor 0): normal logic
                    if (dx > DEADBAND) {
                        motor_destination[0] = dxl_goal_position[1];
                        setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
                    } else if (dx < -DEADBAND) {
                        motor_destination[0] = dxl_goal_position[0];
                        setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
                    } else {
                        motor_destination[0] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2;
                        setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_OFF);
                    }

                    // Y axis (motor 1): INVERTED logic
                    if (dy > DEADBAND) {
                        motor_destination[1] = dxl_goal_position[0]; // inverted
                        setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
                    } else if (dy < -DEADBAND) {
                        motor_destination[1] = dxl_goal_position[1]; // inverted
                        setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
                    } else {
                        motor_destination[1] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2;
                        setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_OFF);
                    }

                    // Z axis (motor 2): normal logic
                    if (dz > DEADBAND) {
                        motor_destination[2] = dxl_goal_position[1];
                        setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
                    } else if (dz < -DEADBAND) {
                        motor_destination[2] = dxl_goal_position[0];
                        setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
                    } else {
                        motor_destination[2] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2;
                        setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_OFF);
                    }

                    spdlog::info("TS: dx: {:4.2f}, dy: {:4.2f}, dz: {:4.2f} | Dest: [{:4d}, {:4d}, {:4d}]",
                               dx, dy, dz,
                               motor_destination[0], motor_destination[1], motor_destination[2]);

                    // Keep motors disabled for trackstar mode (original behavior)
                    spdlog::debug("Trackstar mode: MOVE_MOTORS disabled");
                    state = READ_TRACKSTAR;
                }
                break;
            }

            case MOVE_MOTORS:
                for (int j = 0; j < MOTOR_CNT; j++) {
                    if (vMotors[j].setMotorDestination(&groupSyncWrite, motor_destination[j]) != true) {
                        state = ERR;
                        ERR_FLG = 1;
                        break;
                    }
                }
                if (state == ERR) {
                    break;
                }
                dxl_comm_result = groupSyncWrite.txPacket();
                if (dxl_comm_result != COMM_SUCCESS) {
                    spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
                    state = ERR;
                    ERR_FLG = 1;
                }
                groupSyncWrite.clearParam();
                if (state == ERR) {
                    break;
                }
                state = READ_MOTORS;
                break;
            case READ_MOTORS:
                bool exitFlag;

                dxl_error = 0;
                dxl_comm_result = 0;
                do {
                    groupSyncRead.clearParam();
                    // Sync read present position
                    for (int i = 0; i < MOTOR_CNT; i++) {
                        groupSyncRead.addParam(vMotors[i].getMotorID());
                    }
                    dxl_comm_result = groupSyncRead.txRxPacket();
                    if (dxl_comm_result != COMM_SUCCESS) {
                        // printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                        spdlog::error("{}", packetHandler->getTxRxResult(dxl_comm_result));
                        state = ERR;
                        ERR_FLG = 1;
                        for (int i = 0; i < MOTOR_CNT; i++) {
                            if (groupSyncRead.getError(vMotors[i].getMotorID(), &dxl_error)) {
                                spdlog::error("[ID:{:3d}] {}\n", vMotors[i].getMotorID(),
                                              packetHandler->getRxPacketError(dxl_error));
                            }
                        }
                    }

                    // Reset exitFlag to true at the start of each iteration
                    exitFlag = true;

                    for (int i = 0; i < MOTOR_CNT; i++) {
                        // Check if each motor is at its destination
                        // If any motor is not at destination, set exitFlag to false
                        // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t", vMotors[i].getMotorID(),
                        //        motor_destination[i], vMotors[i].checkAndGetPresentPosition(&groupSyncRead));
                        vMotors[i].checkAndGetPresentPosition(&groupSyncRead);
                        if (!vMotors[i].checkIfAtGoalPosition(motor_destination[i])) {
                            exitFlag = false;
                        }
                    }
                    // printf("\n");
                    if (exitFlag == false) {
                        break;
                    }
                    // Optional: Add a small delay to prevent tight looping
                    // usleep(10000); // 10ms delay, adjust as needed
                } while (exitFlag); // Continue loop until all motors reach destination

                if (state == ERR) {
                    break;
                }
                state = READ_TRACKSTAR;
                break;
            case ERR:
                spdlog::error("User error");
                state = END;
                break;
            case END:
                state = CLEANUP_DAQ;
                break;
            case CLEANUP_DAQ:
                setAllLEDs(daqSystem.digitalTask, LED_OFF);
                DAQStop(daqSystem.analogHandle);
                cleanupDAQSystem(daqSystem);

                state = CLEANUP_MOTORS;
                break;
            case CLEANUP_MOTORS:
                groupSyncWrite.clearParam();
                for (auto motor: vMotors) {
                    motor.disableTorque(packetHandler, portHandler);
                }
            // vMotors.clear();
                portHandler->closePort();
                state = CLEANUP_TRACKSTAR;
                break;

            case CLEANUP_TRACKSTAR:
                cleanUpAndExit();
                cleanupSharedMemory(); // Clean up shared memory
                if (ERR_FLG != 0) {
                    return EXIT_FAILURE;
                }
                return EXIT_SUCCESS;
            default:
                state = ERR;
                break;
        }
    }
}
