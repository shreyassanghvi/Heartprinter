#ifndef MOTOR_H
#define MOTOR_H

#include <string>
#include "../include/dynamixel_sdk/dynamixel_sdk.h"
#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     5                  // DYNAMIXEL moving status threshold


struct DynamixelMotor {
    int ADDR_TORQUE_ENABLE;
    int ADDR_GOAL_POSITION;
    int ADDR_PRESENT_POSITION;
    int LEN_GOAL_POSITION;
    int LEN_PRESENT_POSITION;
    int DXL_ID;
    int DXL_MINIMUM_POSITION_VALUE;
    int DXL_MAXIMUM_POSITION_VALUE;
    int DXL_PRESENT_POSITION_VALUE;
};

typedef DynamixelMotor DynamixelMotor;

class Motor {
public:
    Motor(int id, const std::string &motorType);

    // Add other public members as needed
    int getMotorID() const;

    int getAddrTorqueEnable() const;

    int getAddrGoalPosition() const;

    int getAddrPresentPosition() const;

    int getLenGoalPosition() const;

    int getLenPresentPosition() const;

    int getDxlMinimumPositionValue() const;

    int getDxlMaximumPositionValue() const;

    int getDxlPresentPositionValue() const;

    int enableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    int disableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler);

    //    void disconnectFromMotor();
    bool addGroupSyncRead(dynamixel::GroupSyncRead *groupSyncRead);

    bool addGroupSyncWrite(dynamixel::GroupSyncWrite *groupSyncWrite, int goalPosition);

    uint32_t checkAndGetPresentPosition(dynamixel::GroupSyncRead *groupSyncRead);
    bool checkIfAtGoalPosition(int goalPosition);

private:
    // Add private members as needed
    DynamixelMotor motor{};
};

#endif //MOTOR_H
