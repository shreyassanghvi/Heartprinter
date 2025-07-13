#ifndef MOTOR_H
#define MOTOR_H

#include "../Dynamixel_SDK/dynamixel_sdk.h"
#include "log.h"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque
#define DXL_MOVING_STATUS_THRESHOLD     5                   // DYNAMIXEL moving status threshold
#define VELOCITY_CONTROL_MODE           1                   // Mode is unavailable in Protocol 1.0 Reset
#define POSITION_CONTROL_MODE           3                   // Mode is unavailable in Protocol 1.0 Reset
#define EXTENDED_POSITION_CONTROL_MODE  4                   // Mode is unavailable in Protocol 1.0 Reset
#define PWM_CONTROL_MODE                16                  // Mode is unavailable in Protocol 1.0 Reset
#define ADDR_OPERATING_MODE             11                  // Control table address is different in Dynamixel model
#define ADDR_LED                        65                  // Control table address is different in Dynamixel model

// #define for various definitions for the DYNAMIXEL
#define PROTOCOL_VERSION                2.0                 // See which protocol version is used in the DYNAMIXEL
#define DEVICENAME                      "COM4"      // ex) Windows: "COM1"   Linux: "/dev/ttyUSB0" Mac: "/dev/tty.usbserial-*"



#define BAUDRATE                        57600                // Baudrate of Dynamixel
#define X_SERIES


#ifdef X_SERIES
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4
#define DXL_MINIMUM_POSITION_VALUE 0
#define DXL_MAXIMUM_POSITION_VALUE 4095
#endif

#ifdef MX_SERIES
#define ADDR_TORQUE_ENABLE 64
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_POSITION 132
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4
#define DXL_MINIMUM_POSITION_VALUE 0
#define DXL_MAXIMUM_POSITION_VALUE 4095
#endif

#ifdef PRO_SERIES
#define ADDR_TORQUE_ENABLE 562
#define ADDR_GOAL_POSITION 596
#define ADDR_PRESENT_POSITION 611
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4
#define DXL_MINIMUM_POSITION_VALUE -150000
#define DXL_MAXIMUM_POSITION_VALUE 150000
#endif

#ifdef P_SERIES
#define ADDR_TORQUE_ENABLE 512
#define ADDR_GOAL_POSITION 564
#define ADDR_PRESENT_POSITION 580
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4
#define DXL_MINIMUM_POSITION_VALUE -150000
#define DXL_MAXIMUM_POSITION_VALUE 150000
#endif

#ifdef PRO_A_SERIES
#define ADDR_TORQUE_ENABLE 512
#define ADDR_GOAL_POSITION 564
#define ADDR_PRESENT_POSITION 580
#define LEN_GOAL_POSITION 4
#define LEN_PRESENT_POSITION 4
#define DXL_MINIMUM_POSITION_VALUE -150000
#define DXL_MAXIMUM_POSITION_VALUE 150000
#endif


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

    uint32_t checkAndGetPresentPosition(dynamixel::GroupSyncRead *groupSyncRead);

    bool checkIfAtGoalPosition(int goalPosition) const;

    int setMotorOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                              int operationMode) const;

    void ledOperationMode(dynamixel::PacketHandler *packetHandler, dynamixel::PortHandler *portHandler,
                          int ledStatus) const;

    double mmToDynamixelUnits(double mm) const;

private:
    // Add private members as needed
    DynamixelMotor motor{};

    double pulley_diameter_mm = 30.0;
};

#endif //MOTOR_H
