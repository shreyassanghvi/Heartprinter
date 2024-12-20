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
 * @brief Get the address of the torque enable control table.
 *
 * @return The address of the torque enable control table.
 */
int Motor::getAddrTorqueEnable() const { return motor.ADDR_TORQUE_ENABLE; }
/**
 * @brief Get the address of the goal position control table.
 *
 * @return The address of the goal position control table.
 */
int Motor::getAddrGoalPosition() const { return motor.ADDR_GOAL_POSITION; }
/**
 * @brief Get the address of the present position control table.
 *
 * @return The address of the present position control table.
 */
int Motor::getAddrPresentPosition() const { return motor.ADDR_PRESENT_POSITION; }
/**
 * @brief Get the length of the goal position control table.
 *
 * @return The length of the goal position control table.
 */
int Motor::getLenGoalPosition() const { return motor.LEN_GOAL_POSITION; }
/**
 * @brief Get the length of the present position control table.
 *
 * @return The length of the present position control table.
 */
int Motor::getLenPresentPosition() const { return motor.LEN_PRESENT_POSITION; }
/**
 * @brief Get the minimum position value of the motor.
 *
 * @return The minimum position value of the motor.
 */
int Motor::getDxlMinimumPositionValue() const { return motor.DXL_MINIMUM_POSITION_VALUE; }
/**
 * @brief Get the maximum position value of the motor.
 *
 * @return The maximum position value of the motor.
 */
int Motor::getDxlMaximumPositionValue() const { return motor.DXL_MAXIMUM_POSITION_VALUE; }

/**
 * @brief Get the present position value of the motor.
 *
 * @return The present position value of the motor.
 */
int Motor::getDxlPresentPositionValue() const { return motor.DXL_PRESENT_POSITION_VALUE; }

/**
 * @brief Constructs a Motor object with specified ID and motor type.
 *
 * Sets the motor's control table addresses, data lengths, and position limits based on the given motor type.
 * Supported motor types are "X_SERIES", "MX_SERIES", "PRO_SERIES", "P_SERIES", and "PRO_A_SERIES".
 *
 * @param id The ID of the motor.
 * @param motorType A string representing the type of the motor. Valid values are "X_SERIES", "MX_SERIES", "PRO_SERIES", "P_SERIES", and "PRO_A_SERIES".
 */
Motor::Motor(const int id, const std::string &motorType) {
    this->motor.DXL_ID = id;
    if (motorType == "X_SERIES" || motorType == "MX_SERIES") {
        this->motor.ADDR_TORQUE_ENABLE = 64;
        this->motor.ADDR_GOAL_POSITION = 116;
        this->motor.ADDR_PRESENT_POSITION = 132;
        this->motor.LEN_GOAL_POSITION = 4;
        this->motor.LEN_PRESENT_POSITION = 4;
        this->motor.DXL_MINIMUM_POSITION_VALUE = 0;
        this->motor.DXL_MAXIMUM_POSITION_VALUE = 4095;

    } else if (motorType == "PRO_SERIES") {
        this->motor.ADDR_TORQUE_ENABLE = 562;
        this->motor.ADDR_GOAL_POSITION = 596;
        this->motor.ADDR_PRESENT_POSITION = 611;
        this->motor.LEN_GOAL_POSITION = 4;
        this->motor.LEN_PRESENT_POSITION = 4;
        this->motor.DXL_MINIMUM_POSITION_VALUE = -150000;
        this->motor.DXL_MAXIMUM_POSITION_VALUE = 150000;
    } else if (motorType == "P_SERIES" || motorType == "PRO_A_SERIES") {
        this->motor.ADDR_TORQUE_ENABLE = 512;
        this->motor.ADDR_GOAL_POSITION = 564;
        this->motor.ADDR_PRESENT_POSITION = 580;
        this->motor.LEN_GOAL_POSITION = 4;
        this->motor.LEN_PRESENT_POSITION = 4;
        this->motor.DXL_MINIMUM_POSITION_VALUE = -150000;
        this->motor.DXL_MAXIMUM_POSITION_VALUE = 150000;
    }
}
/**
 * @brief Enable the torque of the motor.
 *
 * @param packetHandler The packet handler to use to communicate with the motor.
 * @param portHandler The port handler to use to communicate with the motor.
 * @return The error code from the communication.
 */
int Motor::enableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) {
    //TODO: add parameter check
    int dxl_comm_result = COMM_TX_FAIL;
    uint8_t dxl_error = 0; // Dynamixel error

    // Write to the torque enable control table to enable the torque of the motor.
    // The address of the torque enable control table is defined in the motor control table.
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), this->getAddrTorqueEnable(),
                                                    TORQUE_ENABLE, &dxl_error);

    // If there was an error with the communication, print the error message.
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    // If there was an error with the packet, print the error message.
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    }
    // If there was no error, print a success message.
    else {
        printf("Dynamixel#%d has been successfully connected \n", this->getMotorID());
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
int Motor::disableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) {
    //TODO: add parameter check
    int dxl_comm_result = COMM_TX_FAIL;             // Communication result
    uint8_t dxl_error = 0;                          // Dynamixel error

    // Write to the torque enable control table to disable the torque of the motor.
    // The address of the torque enable control table is defined in the motor control table.
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, this->getMotorID(), this->getAddrTorqueEnable(),
                                                   TORQUE_DISABLE,
                                                   &dxl_error);

    // If there was an error with the communication, print the error message.
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
    }
    // If there was an error with the packet, print the error message.
    else if (dxl_error != 0) {
        printf("%s\n", packetHandler->getRxPacketError(dxl_error));
    } else
      {
        printf("Dynamixel#%d has been successfully disconnected\n", this->getMotorID());
    }

    // Return the error code from the communication.
    return dxl_error;
}

/**
 * @brief Add this Motor object to the GroupSyncRead object.
 * @param groupSyncRead The GroupSyncRead object to add this Motor object to.
 * @return True if the Motor object is successfully added to the GroupSyncRead object.
 */
bool Motor::addGroupSyncRead(dynamixel::GroupSyncRead *groupSyncRead){
    //TODO: add parameter check
  if (groupSyncRead == nullptr || groupSyncRead->addParam(this->getMotorID())==false) {
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
bool Motor::addGroupSyncWrite(dynamixel::GroupSyncWrite *groupSyncWrite, int goalPosition)
{
  //TODO: add parameter check
    // Allocate goal position value into byte array
    uint8_t param_goal_position[4];
    param_goal_position[0] = DXL_LOBYTE(DXL_LOWORD(goalPosition));
    param_goal_position[1] = DXL_HIBYTE(DXL_LOWORD(goalPosition));
    param_goal_position[2] = DXL_LOBYTE(DXL_HIWORD(goalPosition));
    param_goal_position[3] = DXL_HIBYTE(DXL_HIWORD(goalPosition));
    if(groupSyncWrite == nullptr || groupSyncWrite->addParam(this->getMotorID(), param_goal_position)==false) {
        fprintf(stderr, "[ID:%03d] groupSyncWrite addparam failed", this->getMotorID());
        return false;
    }
    return true;
}
   uint32_t Motor::checkAndGetPresentPosition(dynamixel::GroupSyncRead *groupSyncRead) {
    //TODO: add parameter check

            if (groupSyncRead->
                    isAvailable(this->getMotorID(), this->getAddrPresentPosition(), this->getLenPresentPosition()) != true) {
                fprintf(stderr, "[ID:%03d] groupSyncRead getdata failed", this->getMotorID());
                return EXIT_FAILURE;
            }
            this->motor.DXL_PRESENT_POSITION_VALUE = groupSyncRead->getData(this->getMotorID(), this->getAddrPresentPosition(), this->getLenPresentPosition());
            return this->motor.DXL_PRESENT_POSITION_VALUE;
            }

bool Motor::checkIfAtGoalPosition(int goalPosition){
              return (abs(this->motor.DXL_PRESENT_POSITION_VALUE - goalPosition) > DXL_MOVING_STATUS_THRESHOLD); }