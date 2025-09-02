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

namespace fs = std::filesystem;
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
        spdlog::info("Succeeded to open the port!\n");
    } else {
        spdlog::error("Failed to open the port!\n");
        spdlog::error("Press any key to terminate...\n");
        _getch();
        return EXIT_FAILURE;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        spdlog::info("Succeeded to change the baudrate!\n");
    } else {
        spdlog::error("Failed to change the baudrate!\n");
        spdlog::error("Press any key to terminate...\n");
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
        spdlog::error("Failed to enable torque for motor {}", motors.getMotorID());
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
            spdlog::warn("Invalid data or sample count in DataHandler!\n");
            return;
        }
        // printf("Test\n");
        // printf("Received %u samples. First: %.3f\n", numSamples, data[0]);
        // if (data == nullptr) {
        //     printf("No data\n");
        //     return;
        // }
        std::vector vData(data, data + numSamples);
        spdlog::info("DAQ - Sample Count: {4u} - Received Data[0] - {8.5f}",
                     numSamples,
                     data[0]);
    } catch (std::exception &e) {
        spdlog::error("Error: %s\n", e.what());

        state = ERR;
    }
}

void ErrorHandlerDAQ(const char *errorMessage) {
    spdlog::error("DAQ Error: {}", errorMessage);
    state = ERR;
}

std::shared_ptr<spdlog::logger> create_dated_logger(bool make_default) {
    std::filesystem::create_directories("logs");

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
    oss << "logs/log_"
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
        spdlog::set_pattern("[%Y-%m-%d %H:%M:%S.%e] [%^%L%$]: %v"); // Coloring enabled with %^
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
                    spdlog::error("Failed to set all LEDs off\n");
                    // printLog(LOG_ERROR, buf);
                    state = ERR;
                    break;
                }
                sleep(1);
                if (setAllLEDs(daqSystem.digitalTask, LED_OFF) != EXIT_SUCCESS) {
                    spdlog::error("Failed to set all LEDs off\n");
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
                    spdlog::error("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
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
                spdlog::info("TODO: Initializing load cell...\n");
                DAQStart(daqSystem.analogHandle);
                state = READ_TRACKSTAR;
                break;

            case READ_TRACKSTAR:
                if (data_count == 0) {
                    // printLog(LOG_INFO, "=====================================\n");
                    spdlog::info("Collect %4d Data records from Sensors",RECORD_CNT);

                    spdlog::info("Metric mode was selected, position is in mm.");
                    // printLog(LOG_INFO, "\t-----------\n");
                }
                if (data_count < RECORD_CNT) {
                    for (short i = 0; i < getConnectedSensors(); i++) {
                        retRecord = readATI(i);
                        data_count++;
                        std::string sensorName;
                        switch (i) {
                            case LEFT_SENSOR:
                                sensorName = "Left";
                                break;
                            case RIGHT_SENSOR:
                                sensorName = "Right";
                                break;
                            case BASE_SENSOR:
                                sensorName = "Base";
                                break;
                            case MOVING_BASE_SENSOR:
                                sensorName = "MP";
                                break;
                            default:
                                sensorName = "Unknown";
                                spdlog::error("Incorrect sensor ID");
                                state = ERR;
                                break;
                        }

                        spdlog::info("TS: {:4d} [{}] {:8.3f} {:8.3f} {:8.3f}: {:8.2f} {:8.2f} {:8.2f}",
                                     data_count,
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

                double dx = retRecord.x - REF_X;
                double dy = retRecord.y - REF_Y;
                double dz = retRecord.z - REF_Z;

                // X axis (motor 0): normal logic
                if (dx > DEADBAND) {
                    motor_destination[0] = dxl_goal_position[1]; // forward
                    // setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
                } else if (dx < -DEADBAND) {
                    motor_destination[0] = dxl_goal_position[0]; // backward
                    // setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
                } else {
                    motor_destination[0] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2; // neutral
                    // setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_OFF);
                }

                // Y axis (motor 1): INVERTED logic
                if (dy > DEADBAND) {
                    motor_destination[1] = dxl_goal_position[0]; // backward (inverted)
                    setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
                } else if (dy < -DEADBAND) {
                    motor_destination[1] = dxl_goal_position[1]; // forward (inverted)
                    setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
                } else {
                    motor_destination[1] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2; // neutral
                    setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_OFF);
                }

                // Z axis (motor 2): normal logic
                if (dz > DEADBAND) {
                    motor_destination[2] = dxl_goal_position[1]; // forward
                    setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
                } else if (dz < -DEADBAND) {
                    motor_destination[2] = dxl_goal_position[0]; // backward
                    setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
                } else {
                    motor_destination[2] = (dxl_goal_position[0] + dxl_goal_position[1]) / 2; // neutral
                    setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_OFF);
                }

                spdlog::info("dx: {:.2f}, dy: {:.2f}, dz: {:.2f} | Dest: [{}, {}, {}]",
                             dx, dy, dz,
                             motor_destination[0], motor_destination[1], motor_destination[2]);

                spdlog::warn("State: MOVE_MOTORS disabled");
                state = READ_TRACKSTAR;
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
                    printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
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
                    // groupSyncRead.clearParam();
                    // Sync read present position
                    dxl_comm_result = groupSyncRead.txRxPacket();
                    if (dxl_comm_result != COMM_SUCCESS) {
                        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                        state = ERR;
                        ERR_FLG = 1;
                        for (int i = 0; i < MOTOR_CNT; i++) {
                            if (groupSyncRead.getError(vMotors[i].getMotorID(), &dxl_error)) {
                                spdlog::error("[ID:%03d] %s\n", vMotors[i].getMotorID(),
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
