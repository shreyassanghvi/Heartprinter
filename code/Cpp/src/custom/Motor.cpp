//
// Created by shreyas on 12/19/24.
//
#include "../../include/custom/Motor.h"

/**
 * @brief Get the motor ID.
 *
 * @return The motor ID.
 */
int Motor::getMotorID() const { return motor.DXL_ID; }

/**
 * @brief Default constructor for the Motor class.
 *
 * Initializes the motor ID to -1. The motor object is not valid for use until
 * a valid motor ID is set using the Motor constructor with an ID argument.
 */
Motor::Motor() {
    this->motor.DXL_ID = -1;
}
/**
 * @brief Constructs a Motor object with specified ID and motor type.
 *
 * Sets the motor's control table addresses, data lengths, and position limits based on the given motor type.
 * Supported motor types are "X_SERIES", "MX_SERIES", "PRO_SERIES", "P_SERIES", and "PRO_A_SERIES".
 *
 * @param id The ID of the motor.
 * @param motorType A string representing the type of the motor. Valid values are "X_SERIES", "MX_SERIES", "PRO_SERIES", "P_SERIES", and "PRO_A_SERIES".
 */
Motor::Motor(const int id) {
    this->motor.DXL_ID = id;
}

/**
 * @brief Enable the torque of the motor.
 *
 * @param packetHandler The packet handler to use to communicate with the motor.
 * @param portHandler The port handler to use to communicate with the motor.
 * @return The error code from the communication.
 */
int Motor::enableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) const {
    //TODO: add parameter check
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0; // Dynamixel error

    // Write to the torque enable control table to enable the torque of the motor.
    // The address of the torque enable control table is defined in the motor control table.
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), ADDR_TORQUE_ENABLE,
                                                    TORQUE_ENABLE, &dxl_error);

    // If there was an error with the communication, print the error message.
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    // If there was an error with the packet, print the error message.
    else if (dxl_error != 0) {
        fprintf(stderr, "%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    // If there was no error, print a success message.
    else {
        printf("[info]: Dynamixel#%d Torque has been enabled \n", this->getMotorID());
        ledOperationMode(packetHandler, portHandler, LED_ON);
    }

    // Return the error code from the communication.
    return dxl_error;
}

/**
 * @brief Disable the torque of the motor.
 *
 * @param packetHandler The packet handler to use to communicate with the motor.
 * @param portHandler The port handler to use to communicate with the motor.
 * @return The error code from the communication.
 */
int Motor::disableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) const {
    //TODO: add parameter check
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0; // Dynamixel error

    // Write to the torque enable control table to disable the torque of the motor.
    // The address of the torque enable control table is defined in the motor control table.
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), ADDR_TORQUE_ENABLE,
                                                    TORQUE_DISABLE,
                                                    &dxl_error);

    // If there was an error with the communication, print the error message.
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    // If there was an error with the packet, print the error message.
    else if (dxl_error != 0) {
        fprintf(stderr, "%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        printf("[info]: Dynamixel#%d Torque has been disabled\n", this->getMotorID());
        ledOperationMode(packetHandler, portHandler, LED_OFF);
    }

    // Return the error code from the communication.
    return dxl_error;
}

/**
 * @brief Add this Motor object to the GroupSyncRead object.
 * @param groupSyncRead The GroupSyncRead object to add this Motor object to.
 * @return True if the Motor object is successfully added to the GroupSyncRead object.
 */
bool Motor::addGroupSyncRead(dynamixel::GroupSyncRead *groupSyncRead) {
    //TODO: add parameter check
    if (groupSyncRead == nullptr || groupSyncRead->addParam(this->getMotorID()) == false) {
        fprintf(stderr, "[ID:%03d] groupSyncRead addparam failed", this->getMotorID());
        return false;
    }
    return true;
}

/**
 * @brief Add this Motor object to the GroupSyncWrite object.
 *
 * @param groupSyncWrite The GroupSyncWrite object to add this Motor object to.
 * @param goalPosition The goal position to write to the motor.
 * @return True if the Motor object is successfully added to the GroupSyncWrite object.
 */
bool Motor::addGroupSyncWrite(dynamixel::GroupSyncWrite *groupSyncWrite, int goalPosition) {
    //TODO: add parameter check
    // Allocate goal position value into byte array
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPosition));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPosition));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPosition));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPosition));
    if (groupSyncWrite->addParam(this->getMotorID(), param_goal_position) == false) {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", this->getMotorID());
        return false;
    }
    return true;
}

uint32_t Motor::checkAndGetPresentPosition(dynamixel::GroupSyncRead *groupSyncRead) {
    //TODO: add parameter check

    if (groupSyncRead->
        isAvailable(this->getMotorID(), ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION) != true) {
        fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->getMotorID());
        return EXIT_FAILURE;
    }
    this->motor.DXL_PRESENT_POSITION_VALUE = groupSyncRead->getData(this->getMotorID(), ADDR_PRESENT_POSITION,
                                                                    LEN_PRESENT_POSITION);
    return this->motor.DXL_PRESENT_POSITION_VALUE;
}

bool Motor::checkIfAtGoalPosition(int goalPosition) const {
    return (abs(this->motor.DXL_PRESENT_POSITION_VALUE - goalPosition) > DXL_MOVING_STATUS_THRESHOLD);
}

int Motor::setMotorOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                                 int operationMode) const {
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0; // Dynamixel error
    if (operationMode != EXTENDED_POSITION_CONTROL_MODE && operationMode != VELOCITY_CONTROL_MODE && operationMode != PWM_CONTROL_MODE) {
        operationMode = POSITION_CONTROL_MODE;
    }
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), ADDR_OPERATING_MODE,
                                                    operationMode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        switch (operationMode) {
            case EXTENDED_POSITION_CONTROL_MODE:
                printf("[info]: Operating mode changed to extended position control mode. \n");
                break;
            case VELOCITY_CONTROL_MODE:
                printf("[info]: Operating mode changed to velocity control mode. \n");
                break;
            case PWM_CONTROL_MODE:
                printf("[info]: Operating mode changed to PWM control mode. \n");
                break;
            default:
                printf("[info]: Operating mode changed to position control mode. \n");
                break;
        }
    }
    return operationMode;
}

void Motor::ledOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                             int ledStatus) const {
    int dxl_comm_result = COMM_TX_FAIL; // Communication result
    uint8_t dxl_error = 0; // Dynamixel error
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), ADDR_LED,
                                                    ledStatus, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS) {
        fprintf(stderr, "%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    } else if (dxl_error != 0) {
        fprintf(stderr, "%s\n", packetHandler->getRxPacketError(dxl_error));
    } else {
        printf("[info]: LED turned %s. \n", ledStatus == LED_ON ? "ON" : "OFF");
    }
}
