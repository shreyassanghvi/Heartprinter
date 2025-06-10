//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/init.h"
#include "../../include/custom/ATC3DG.h"

// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
// #define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"


// #define BAUDRATE                        57600
#define ESC_ASCII_VALUE                 0x1b                // ASCII value for the ESC key

//user defined #define
#define RECORD_CNT                          1000                 // Number of records to collect
#define MOTOR_CNT                           3                  // Number of motors to control
#define SENSOR_ID_LEFT                      0                  // Sensor ID to use for left
#define SENSOR_ID_BASE                      1                  // Sensor ID to use for base
#define SENSOR_ID_RIGHT                     2                  // Sensor ID to use for right
#define SENSOR_ID_INJECTOR                  3                  // Sensor ID to use for injector


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


int initMotors(Motor motors) {
    uint8_t dxl_error = 0; // Dynamixel error
    // Open port
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
    dxl_error = 0;

    motors.setMotorOperationMode(packetHandler, portHandler, EXTENDED_POSITION_CONTROL_MODE);
    // Enable Dynamixel#i Torque
    dxl_error += motors.enableTorque(packetHandler, portHandler);

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
    if (errorCode != BIRD_ERROR_SUCCESS) CTrackstar::errorHandler(errorCode, __LINE__);

    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////
    //
    // Turn off the transmitter before exiting
    // We turn off the transmitter by "selecting" a transmitter with an id of "-1"
    //
    short id = -1;
    errorCode = SetSystemParameter(SELECT_TRANSMITTER, &id, sizeof(id));
    if (errorCode != BIRD_ERROR_SUCCESS) CTrackstar::errorHandler(errorCode, __LINE__);
}

//Init commit for Control loop code
int main(int argc, char *argv[]) {
    STATES state = START;
    // Motor motors[MOTOR_CNT];
    std::vector<Motor> vMotors;
    int dxl_comm_result; // Communication result
    int dxl_goal_position[2] = {DXL_MINIMUM_POSITION_VALUE, static_cast<int>(DXL_MAXIMUM_POSITION_VALUE * 1.5)};
    uint8_t dxl_error; // Dynamixel error
    long data_count = 0;
    int ERR_FLG = 0;
    int motor_destination[MOTOR_CNT];

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
                break;
            case INIT_MOTORS:
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
                CTrackstar::initTxRx();
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
                    retRecord = CTrackstar::readATI(SENSOR_ID_LEFT);
                    data_count++;
                    printf("%4ld [%d] %8.3f %8.3f %8.3f: %8.2f %8.2f %8.2f\n",
                           data_count,
                           SENSOR_ID_LEFT,
                           retRecord.x, retRecord.y, retRecord.z,
                           retRecord.a, retRecord.e, retRecord.r
                    );

                    state = SET_MOTOR;
                } else {
                    printf("=====================================\n");
                    state = END;
                }

                break;
            case SET_MOTOR:
                if (retRecord.x < 30 && retRecord.x > -30) {
                    motor_destination[0] = dxl_goal_position[1];
                } else {
                    motor_destination[0] = dxl_goal_position[0];
                }
                if (retRecord.y < 30 && retRecord.y > -30) {
                    motor_destination[1] = dxl_goal_position[1];
                } else {
                    motor_destination[1] = dxl_goal_position[0];
                }
                if (retRecord.z < 30 && retRecord.z > -30) {
                    motor_destination[2] = dxl_goal_position[1];
                } else {
                    motor_destination[2] = dxl_goal_position[0];
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
                groupSyncWrite.clearParam();
                for (auto motor: vMotors) {
                    motor.disableTorque(packetHandler, portHandler);
                }
            // vMotors.clear();
                portHandler->closePort();
                state = CLEANUP_MOTORS;
                break;

            case CLEANUP_MOTORS:

                state = CLEANUP_TRACKSTAR;
                break;
            case CLEANUP_TRACKSTAR:
                cleanUpAndExit();
                if (ERR_FLG != 0) {
                    return EXIT_FAILURE;
                }
                return EXIT_SUCCESS;


            default:
                break;
        }
    }
    // // collect as many records as specified in the command line


    // printf("      -----------\n");
    // for (int i = 0; i < RECORD_CNT; i++) {
    //     short sensorID = 0;
    //     retRecord = readATI(sensorID);
    //     printf("%4d [%d] %8.3f %8.3f %8.3f: %8.2f %8.2f %8.2f\n",
    //            i + 1,
    //            sensorID,
    //            retRecord.x, retRecord.y, retRecord.z,
    //            retRecord.a, retRecord.e, retRecord.r
    //     );
    //     // while (true) {
    //     // printf("Press any key to continue! (or press ESC to quit!)\n");
    //     // if (getch() == ESC_ASCII_VALUE)
    //     //     break;
    //     // Add Dynamixel#1 goal position value to the Syncwrite storage
    //     int motorPos[3] = {1, 2, 3};
    //     if (retRecord.x < 30 || retRecord.x > -30) {
    //         motorPos[0] = dxl_goal_position[0];
    //     } else {
    //         motorPos[0] = dxl_goal_position[1];
    //     }
    //     if (retRecord.y < 30) {
    //         motorPos[1] = dxl_goal_position[0];
    //     } else {
    //         motorPos[1] = dxl_goal_position[1];
    //     }
    //     if (retRecord.z < 30) {
    //         motorPos[2] = dxl_goal_position[0];
    //     } else {
    //         motorPos[2] = dxl_goal_position[1];
    //     }
    //
    //
    //     for (int j = 0; j < MOTOR_CNT; j++) {
    //         if (vMotors[j].addGroupSyncWrite(&groupSyncWrite, motorPos[index]) != true) {
    //             printf("GroupSyncWrite addGroupSyncWrite failed!\n");
    //             break;
    //         }
    //     }
    //     // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    //     // if (dxl2.addGroupSyncWrite(&groupSyncWrite, dxl_goal_position[index]) != true) {
    //     //     return EXIT_FAILURE;
    //     // }
    //
    //     // Syncwrite goal position
    //     dxl_comm_result = groupSyncWrite.txPacket();
    //     if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //
    //     // Clear syncwrite parameter storage
    //     groupSyncWrite.clearParam();
    //     bool exitFlag;
    //
    //     dxl_error = 0;
    //     do {
    //         // Syncread present position
    //         dxl_comm_result = groupSyncRead.txRxPacket();
    //         exitFlag = true;
    //         for (int i = 0; i < MOTOR_CNT; i++) {
    //             if (dxl_comm_result != COMM_SUCCESS) {
    //                 printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    //             } else {
    //                 if (groupSyncRead.getError(vMotors[i].getMotorID(), &dxl_error)) {
    //                     printf("[ID:%03d] %s\n", vMotors[i].getMotorID(), packetHandler->getRxPacketError(dxl_error));
    //                 }
    //             }
    //         }
    //
    //         for (int i = 0; i < MOTOR_CNT; i++) {
    //             // printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t", vMotors[i].getMotorID(),
    //             // dxl_goal_position[index], vMotors[i].checkAndGetPresentPosition(&groupSyncRead));
    //             exitFlag &= vMotors[i].checkIfAtGoalPosition(dxl_goal_position[index]);
    //         }
    //         // printf("\n");
    //     } while (exitFlag);
    //     //
    //     // // Change goal position
    //     // index = (index + 1) % 2;
    // }
    //
    // cleanUpAndExit();
    //
    //
    // for (int i = 0; i < MOTOR_CNT; i++) {
    //     // // Disable Dynamixel#1 Torque
    //     vMotors[i].disableTorque(packetHandler, portHandler);
    // }
    //
    // // Close port
    // portHandler->closePort();
    //
    // if (dxl_comm_result != COMM_SUCCESS) {
    //     return EXIT_FAILURE;
    // }
    // return EXIT_SUCCESS;
}
