#ifndef MOTOR_H
#define MOTOR_H

#include "../Dynamixel_SDK/dynamixel_sdk.h"

#include <spdlog/spdlog.h>

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     10                   // DYNAMIXEL moving status threshold
#define VELOCITY_CONTROL_MODE           1                   // Mode is unavailable in Protocol 1.0 Reset
#define POSITION_CONTROL_MODE           3                   // Mode is unavailable in Protocol 1.0 Reset
#define EXTENDED_POSITION_CONTROL_MODE  4                   // Mode is unavailable in Protocol 1.0 Reset
#define PWM_CONTROL_MODE                16                  // Mode is unavailable in Protocol 1.0 Reset
#define ADDR_OPERATING_MODE             11                  // Control table address is different in Dynamixel model
#define ADDR_LED                        65                  // Control table address is different in Dynamixel model

// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION 2.0                 // See which protocol version is used in the DYNAMIXEL

#define ADDR_TORQUE_ENABLE         64
#define ADDR_GOAL_POSITION         116
#define ADDR_PRESENT_POSITION      132
#define LEN_GOAL_POSITION          4
#define LEN_PRESENT_POSITION       4
#define DXL_MINIMUM_POSITION_VALUE 0
#define DXL_MAXIMUM_POSITION_VALUE 4095

#define ADDR_VELOCITY_PROFILE        112
#define DXL_VELOCITY_PROFILE_VALUE   375  // ~5 RPM (unit: 0.229 rpm)

struct DynamixelMotor {
    int DXL_ID;
    int DXL_PRESENT_POSITION_VALUE;
};

typedef DynamixelMotor DynamixelMotor;

class Motor {
public:
    Motor();

    Motor(int id);

    // Add other public members as needed
    int getMotorID() const;

    int enableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) const;

    int disableTorque(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler) const;

    bool setMotorDestination(dynamixel::GroupSyncWrite *groupSyncWrite, double goalPosition) const;

    uint32_t checkAndGetPresentPosition(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler, dynamixel::GroupSyncRead *groupSyncRead);

    bool checkIfAtGoalPosition(int goalPosition) const;

    int setMotorOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                              int operationMode) const;

    int setVelocityProfile(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                         uint32_t velocityProfile) const;

    void ledOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                          int ledStatus) const;

    double mmToDynamixelUnits(double mm) const;

private:
    // Add private members as needed
    DynamixelMotor motor{};

    double pulley_diameter_mm = 7.5;
};

#endif //MOTOR_H
