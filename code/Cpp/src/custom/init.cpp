//
// Created by shreyas on 12/19/24.
//
#if defined(__linux__) || defined(__APPLE__)
#include <fcntl.h>
#include <termios.h>
#define STDIN_FILENO 0
#elif defined(_WIN32) || defined(_WIN64)
#include <conio.h>
#endif

#include "../../include/custom/Motor.h"
#include "../../src/DynamixelSDK.h"

#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
#define DEVICENAME                      "/dev/ttyUSB0"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"



#define BAUDRATE                        57600
#define ESC_ASCII_VALUE                 0x1b                // ASCII value for the ESC key

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


/**
 * @brief Check if a key is pressed on the console.
 * @return 1 if a key is pressed, 0 if not.
 *
 * This function is used to check if a key is pressed on the console, and is
 * used by the example programs to check if the user wants to exit.
 *
 * Note: This function is not portable and may not work on all systems.
 */
int kbhit(void) {
#if defined(__linux__) || defined(__APPLE__)
    termios oldt{}, newt{};

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    int ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }

    return 0;
#elif defined(_WIN32) || defined(_WIN64)
    return _kbhit();
#endif
}
// /**
//  * @brief Main function to control Dynamixel motors using the Dynamixel SDK.
//  *
//  * This function initializes the communication with Dynamixel motors, enabling torque,
//  * setting goal positions, and reading present positions in a loop. It utilizes
//  * GroupSyncWrite to synchronize writing goal positions and GroupSyncRead to read
//  * present positions of multiple motors. The function continues to operate until the
//  * ESC key is pressed, toggling between two goal positions. After exiting the loop,
//  * it disables the torque and closes the communication port.
//  *
//  * @return int Returns 0 upon successful execution.
//  */
// int main() {
//     // NOTE: DO NOT USE MOTORS THAT ARE NOT OF THE SAME TYPE if connected on the same PORT as they can cause damage
//     auto dxl1 = Motor(1, "X_SERIES");
//     auto dxl2 = Motor(2, "X_SERIES");
//     // Initialize PortHandler instance
//     // Set the port path
//     // Get methods and members of PortHandlerLinux or PortHandlerWindows
//     dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);
//
//     // Initialize PacketHandler instance
//     // Set the protocol version
//     // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
//     dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
//
//     // Initialize GroupSyncWrite instance
//     dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, dxl1.getAddrGoalPosition(),
//                                              dxl1.getLenGoalPosition());
//     // Initialize Groupsyncread instance for Present Position
//     dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, dxl1.getAddrPresentPosition(),
//                                            dxl1.getLenPresentPosition());
//
//     int index = 0;
//     int dxl_comm_result = COMM_TX_FAIL; // Communication result
//     int dxl_goal_position[2] = {dxl1.getDxlMinimumPositionValue(), dxl1.getDxlMaximumPositionValue()};
//     // Goal position
//
//     uint8_t dxl_error = 0; // Dynamixel error
//     // Open port
//     if (portHandler->openPort()) {
//         printf("Succeeded to open the port!\n");
//     } else {
//         printf("Failed to open the port!\n");
//         printf("Press any key to terminate...\n");
//         getch();
//         return EXIT_FAILURE;
//     }
//
//     // Set port baudrate
//     if (portHandler->setBaudRate(BAUDRATE)) {
//         printf("Succeeded to change the baudrate!\n");
//     } else {
//         printf("Failed to change the baudrate!\n");
//         printf("Press any key to terminate...\n");
//         getch();
//         return EXIT_FAILURE;
//     }
//     // Enable Dynamixel#1 Torque
//     dxl_error = dxl1.enableTorque(packetHandler, portHandler);
//
//     // Enable Dynamixel#2 Torque
//     dxl_error += dxl2.enableTorque(packetHandler, portHandler);
//
//     if (dxl_error != 0) {
//         return EXIT_FAILURE;
//     }
//     // Add parameter storage for Dynamixel#1 present position value
//     if (groupSyncRead.addParam(dxl1.getMotorID()) != true) {
//         return EXIT_FAILURE;
//     }
//     dxl_error =0;
//     // Add parameter storage for Dynamixel#2 present position value
//
//     if (dxl2.addGroupSyncRead(&groupSyncRead) != true) {
//         return EXIT_FAILURE;
//     }
//
//     while (true) {
//         printf("Press any key to continue! (or press ESC to quit!)\n");
//         if (getch() == ESC_ASCII_VALUE)
//             break;
//         // Add Dynamixel#1 goal position value to the Syncwrite storage
//         if (dxl1.addGroupSyncWrite(&groupSyncWrite, dxl_goal_position[index]) != true) {
//             return EXIT_FAILURE;
//         }
//         // Add Dynamixel#2 goal position value to the Syncwrite parameter storage
//         if (dxl2.addGroupSyncWrite(&groupSyncWrite, dxl_goal_position[index]) != true) {
//             return EXIT_FAILURE;
//         }
//
//         // Syncwrite goal position
//         dxl_comm_result = groupSyncWrite.txPacket();
//         if (dxl_comm_result != COMM_SUCCESS) printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//
//         // Clear syncwrite parameter storage
//         groupSyncWrite.clearParam();
//
//         do {
//             dxl_error =0;
//             // Syncread present position
//             dxl_comm_result = groupSyncRead.txRxPacket();
//             if (dxl_comm_result != COMM_SUCCESS) {
//                 printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
//             } else if (groupSyncRead.getError(dxl1.getMotorID(), &dxl_error)) {
//                 printf("[ID:%03d] %s\n", dxl1.getMotorID(), packetHandler->getRxPacketError(dxl_error));
//             } else if (groupSyncRead.getError(dxl2.getMotorID(), &dxl_error)) {
//                 printf("[ID:%03d] %s\n", dxl2.getMotorID(), packetHandler->getRxPacketError(dxl_error));
//             }
//
//             // Check if groupsyncread data of Dynamixel#1 &#2 is available and print
//             printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n", dxl1.getMotorID(),
//                    dxl_goal_position[index], dxl1.checkAndGetPresentPosition(&groupSyncRead), dxl2.getMotorID(),
//                    dxl_goal_position[index],
//                    dxl2.checkAndGetPresentPosition(&groupSyncRead));
//         } while (dxl1.checkIfAtGoalPosition(dxl_goal_position[index]) || dxl2.checkIfAtGoalPosition(
//                      dxl_goal_position[index]));
//
//         // Change goal position
//         index = (index + 1) % 2;
//     }
//
//     // // Disable Dynamixel#1 Torque
//     dxl1.disableTorque(packetHandler, portHandler);
//
//     // Disable Dynamixel#2 Torque
//     dxl2.disableTorque(packetHandler, portHandler);
//
//     // Close port
//     portHandler->closePort();
//     if (dxl_comm_result != COMM_SUCCESS) {
//         return EXIT_FAILURE;
//     }
//     return EXIT_SUCCESS;
// }
