//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/Init.h"
#include "../../include/Trackstar/ATC3DG.h"

// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
// #define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// #define BAUDRATE                        57600

//user defined #define
#define RECORD_CNT                          1000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control
#define SENSOR_ID_LEFT                      0                  // Sensor ID to use for left
#define SENSOR_ID_BASE                      1                  // Sensor ID to use for base
#define SENSOR_ID_RIGHT                     2                  // Sensor ID to use for right
#define SENSOR_ID_INJECTOR                  3                  // Sensor ID to use for injector

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


/**
 * @brief Reads a single character from the console without echoing it.
 * @return The character read, or EOF on failure.
 */
int getch() {
#if defined(__linux__) || defined(__APPLE__)
    termios oldt{}, newt{};
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
#elif defined(_WIN32) || defined(_WIN64)
    return _getch();
#endif
}

int setupMotorPort() {
    if (portHandler->openPort()) {
        printf("Succeeded to open the port!\n");
    } else {
        printf("Failed to open the port!\n");
        printf("Press any key to terminate...\n");
        getch();
        return EXIT_FAILURE;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE)) {
        printf("Succeeded to change the baudrate!\n");
    } else {
        printf("Failed to change the baudrate!\n");
        printf("Press any key to terminate...\n");
        getch();
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}

int initMotors(Motor motors) {
    uint8_t dxl_error = 0; // Dynamixel error
    motors.setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE);
    // Enable Dynamixel#i Torque
    dxl_error += motors.enableTorque(packetHandler, portHandler);
    printf("=============================\n");
    if (dxl_error != 0) {
        return EXIT_FAILURE;
    }

    // Add parameter storage for Dynamixel#1 present position value
    if (groupSyncRead.addParam(motors.getMotorID()) != true) {
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

    printf("Received %u samples. First: %.3f\n", numSamples, data[0]);
}

void ErrorHandler(const char *errorMessage) {
    fprintf(stderr, "DAQ ERROR: %s\n", errorMessage);
    // state = ERR;
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
    DOUBLE_POSITION_ANGLES_RECORD retRecord;
    retRecord.a = 0;
    retRecord.e = 0;
    retRecord.r = 0;

    retRecord.x = 0;
    retRecord.y = 0;
    retRecord.z = 0;
    //TODO: check and fix state machine
    while (true) {
        switch (state) {
            case START:
                dxl_comm_result = COMM_SUCCESS;
                groupSyncWrite.clearParam();
                state = INIT_MOTORS;
                printf("\n=====================================\n");
                break;
            case INIT_MOTORS:
                printf("Initializing motors...");
                printf("\n=====================================\n");
                if (setupMotorPort()) {
                    state = ERR;
                    ERR_FLG = 1;
                    break;
                }
                printf("=============================\n");
                for (int i = 0; i < MOTOR_CNT; i++) {
                    motor_destination[i] = 0;
                    vMotors.emplace_back(i);
                    if (initMotors(vMotors.back()) != EXIT_SUCCESS) {
                        state = ERR;
                        ERR_FLG = 1;
                        break;
                    }
                    if (vMotors.back().addGroupSyncWrite(&groupSyncWrite, DXL_MINIMUM_POSITION_VALUE) != true) {
                        state = ERR;
                        ERR_FLG = 1;
                        break;
                    }
                }
                dxl_comm_result = groupSyncWrite.txPacket();
                if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
                groupSyncWrite.clearParam();
                if (state == ERR) {
                    break;
                }
                state = INIT_TRACKSTAR;
                break;
            case INIT_TRACKSTAR:
                initTxRx();
                state = INIT_DAQ;
                break;
            case INIT_DAQ: {
                printf("=====================================\n");
                printf("Initializing the daq...");
                DigitalConfig digiConfig = {"Dev1", "PFI0:2"};
                AnalogConfig analogConfig = {
                    "Dev1", "ai0",
                    0.0, 10.0,
                    10000.0,
                    1000
                };
                daqSystem = initDAQSystem(digiConfig, analogConfig, DataHandler, ErrorHandler);
                if (!daqSystem.initialized) {
                    printf("Failed to initialize DAQ system\n");
                    state = ERR;
                    break;
                }
                if (setAllLEDs(daqSystem.digitalTask, LED_OFF) != EXIT_SUCCESS) {
                    printf("Failed to set all LEDs off\n");
                    state = ERR;
                    break;
                }


                printf("=====================================\n");
                state = INIT_LOADCELL;
                break;
            }
            case INIT_LOADCELL:
                //TODO: Initialize load cell
                printf("TODO: Initializing load cell...");
                DAQ_Start(daqSystem.analogHandle);
                state = READ_TRACKSTAR;
                break;

            case READ_TRACKSTAR:
                if (data_count == 0) {
                    printf("=====================================\n");
                    printf("Collect %4d Data records from Sensors\n",RECORD_CNT);
                    printf("=====================================\n");
                    printf("Note: Metric mode was selected, position is in mm.\n");
                    printf("\t-----------\n");
                }
                if (data_count < RECORD_CNT) {
                    retRecord = readATI(SENSOR_ID_LEFT);
                    data_count++;
                    printf("%4ld [%d] %8.3f %8.3f %8.3f: %8.2f %8.2f %8.2f\n",
                           data_count,
                           SENSOR_ID_LEFT,
                           retRecord.x, retRecord.y, retRecord.z,
                           retRecord.a, retRecord.e, retRecord.r
                    );

                    state = READ_DAQ;
                } else {
                    printf("=====================================\n");
                    state = END;
                }
                break;
            case READ_DAQ:


                state = SET_MOTOR;
                break;
            case SET_MOTOR:
                if (retRecord.x < 30 && retRecord.x > -30) {
                    motor_destination[0] = dxl_goal_position[1];
                    setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_ON);
                } else {
                    motor_destination[0] = dxl_goal_position[0];
                    setLEDState(daqSystem.digitalTask, LED::LEFT_BASE, LED_OFF);
                }
                if (retRecord.y < 30 && retRecord.y > -30) {
                    motor_destination[1] = dxl_goal_position[1];
                    setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_ON);
                } else {
                    motor_destination[1] = dxl_goal_position[0];
                    setLEDState(daqSystem.digitalTask, LED::CENTER_BASE, LED_OFF);
                }
                if (retRecord.z < 30 && retRecord.z > -30) {
                    motor_destination[2] = dxl_goal_position[1];
                    setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_ON);
                } else {
                    motor_destination[2] = dxl_goal_position[0];
                    setLEDState(daqSystem.digitalTask, LED::RIGHT_BASE, LED_OFF);
                }
                // printf("[ID:%4ld]", data_count);
                // for (int i: motor_destination) {
                //     printf("%d ", i);
                // }
                // printf("\n");
                state = MOVE_MOTORS;
                break;
            case MOVE_MOTORS:
                for (int j = 0; j < MOTOR_CNT; j++) {
                    if (vMotors[j].addGroupSyncWrite(&groupSyncWrite, motor_destination[j]) != true) {
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
                                printf("[ID:%03d] %s\n", vMotors[i].getMotorID(),
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
                fprintf(stderr, "User error\n");
                state = END;
                break;
            case END:
                state = CLEANUP_MOTORS;
                break;

            case CLEANUP_MOTORS:
                groupSyncWrite.clearParam();
                for (auto motor: vMotors) {
                    motor.disableTorque(packetHandler, portHandler);
                }
                // vMotors.clear();
                portHandler->closePort();
                state = CLEANUP_DAQ;
                break;
            case CLEANUP_DAQ:
                DAQ_Stop(daqSystem.analogHandle);
                cleanupDAQSystem(daqSystem);
                state = CLEANUP_TRACKSTAR;
                break;
            case CLEANUP_TRACKSTAR:
                cleanUpAndExit();
                printf("\n=====================================\n");
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
