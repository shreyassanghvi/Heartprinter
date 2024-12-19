#!/usr/bin/sudo python
# !/usr/bin/env python
# -*- coding: utf-8 -*-
import os
# from dynamixel_sdk import * as dxl_class
import dynamixel_sdk as dxl_class

if os.name == 'nt':
    import msvcrt


    def getch():
        return msvcrt.getch().decode()
else:
    import sys, tty, termios

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)


    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


class Dynamexel:
    portHandler = None
    packetHandler = None
    groupSyncWrite = None
    groupSyncRead = None

    @staticmethod
    def close_port_handler():
        print("[info]: Serial Port closed.")
        Dynamexel.portHandler.closePort()

    @staticmethod
    def _init_handlers(self):
        if Dynamexel.portHandler is None:
            Dynamexel.portHandler = dxl_class.PortHandler(DEVICENAME)
            # Open port
            if Dynamexel.portHandler.openPort():
                print("[Info]: Succeeded to open the port")
            else:
                print("[Error]: Failed to open the port")
                print("Press any key to terminate...")
                getch()
                quit()

            # Set port baudrate
            if Dynamexel.portHandler.setBaudRate(self.BAUDRATE):
                print("[info]: Succeeded to change the baudrate")
            else:
                print("Failed to change the baudrate")
                print("Press any key to terminate...")
                getch()
                quit()

        if Dynamexel.packetHandler is None:
            # Initialize PacketHandler instance
            # Set the protocol version
            # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
            Dynamexel.packetHandler = dxl_class.PacketHandler(PROTOCOL_VERSION)
            if Dynamexel.packetHandler is None:
                print("[Error]: packetHandler is not initialized")
                return
        if Dynamexel.groupSyncWrite is None:
            # Initialize GroupSyncWrite instance
            Dynamexel.groupSyncWrite = dxl_class.GroupSyncWrite(Dynamexel.portHandler, Dynamexel.packetHandler,
                                                                self.ADDR_GOAL_POSITION, self.LEN_GOAL_POSITION)
        if Dynamexel.groupSyncRead is None:
            # Initialize GroupSyncRead instance for Present Position
            Dynamexel.groupSyncRead = dxl_class.GroupSyncRead(Dynamexel.portHandler, Dynamexel.packetHandler,
                                                              self.ADDR_PRESENT_POSITION,
                                                              self.LEN_PRESENT_POSITION)

    def __init__(self, dxl_id, model='X_SERIES', device_name='/dev/ttyUSB0', protocol=2.0, target=0):
        self.DXL_ID = dxl_id
        self.MODEL = model
        self.DEVICENAME = device_name
        self.PROTOCOL_VER = protocol
        self.dxl_target = target
        self.TORQUE_ENABLE = 1  # Value for enabling the torque
        self.TORQUE_DISABLE = 0  # Value for disabling the torque
        self.DXL_MOVING_STATUS_THRESHOLD = 20  # Dynamixel moving status threshold
        self._set_control_table_addresses()
        self._init_handlers(self)

    def _set_control_table_addresses(self):
        if self.MODEL in ['X_SERIES', 'MX_SERIES']:
            self.ADDR_TORQUE_ENABLE = 64
            self.ADDR_GOAL_POSITION = 116
            self.LEN_GOAL_POSITION = 4
            self.ADDR_PRESENT_POSITION = 132
            self.LEN_PRESENT_POSITION = 4
            self.DXL_MINIMUM_POSITION_VALUE = 0
            self.DXL_MAXIMUM_POSITION_VALUE = 4095
            self.BAUDRATE = 57600
        elif self.MODEL == 'PRO_SERIES':
            self.ADDR_TORQUE_ENABLE = 562
            self.ADDR_GOAL_POSITION = 596
            self.LEN_GOAL_POSITION = 4
            self.ADDR_PRESENT_POSITION = 611
            self.LEN_PRESENT_POSITION = 4
            self.DXL_MINIMUM_POSITION_VALUE = -150000
            self.DXL_MAXIMUM_POSITION_VALUE = 150000
            self.BAUDRATE = 57600
        elif self.MODEL in ['P_SERIES', 'PRO_A_SERIES']:
            self.ADDR_TORQUE_ENABLE = 512
            self.ADDR_GOAL_POSITION = 564
            self.LEN_GOAL_POSITION = 4
            self.ADDR_PRESENT_POSITION = 580
            self.LEN_PRESENT_POSITION = 4
            self.DXL_MINIMUM_POSITION_VALUE = -150000
            self.DXL_MAXIMUM_POSITION_VALUE = 150000
            self.BAUDRATE = 57600

    def init_dynamixels(self):
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = Dynamexel.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                            self.ADDR_TORQUE_ENABLE,
                                                                            self.TORQUE_ENABLE)
        if dxl_comm_result != dxl_class.COMM_SUCCESS:
            print("%s" % Dynamexel.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % Dynamexel.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[Info]: Dynamixel#%d has been successfully connected" % self.DXL_ID)

        # Add parameter storage for Dynamixel#1 present position value
        dxl_addparam_result = self.groupSyncRead.addParam(self.DXL_ID)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % self.DXL_ID)
            quit()

    def disconnect_dynamixels(self):
        # Enable Dynamixel#1 Torque
        dxl_comm_result, dxl_error = Dynamexel.packetHandler.write2ByteTxRx(self.portHandler, self.DXL_ID,
                                                                            self.ADDR_TORQUE_ENABLE,
                                                                            self.TORQUE_DISABLE)
        if dxl_comm_result != dxl_class.COMM_SUCCESS:
            print("%s" % Dynamexel.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % Dynamexel.packetHandler.getRxPacketError(dxl_error))
        else:
            print("[Info]: Dynamixel#%d has been successfully disconnected" % self.DXL_ID)

    def sync_write_dynamixels(self, send_tx=False):
        if send_tx:
            dxl_comm_result = Dynamexel.groupSyncWrite.txPacket()
            if dxl_comm_result != dxl_class.COMM_SUCCESS:
                print("%s" % Dynamexel.packetHandler.getTxRxResult(dxl_comm_result))
            # Clear syncwrite parameter storage
            Dynamexel.groupSyncWrite.clearParam()
            return dxl_comm_result
        else:
            goal_position = self._convert_val_to_goal_pos(self.dxl_target)
            # Add Dynamixel#1 goal position value to the Syncwrite parameter storage
            if not Dynamexel.groupSyncWrite.addParam(self.DXL_ID, goal_position):
                print("[ID:%03d] groupSyncWrite addparam failed" % self.DXL_ID)
                quit()

    def set_target_pos(self, target):
        self.dxl_target = target

    def sync_read_dynamixels(self, receive_rx=False):
        if receive_rx:
            dxl_comms_connect = Dynamexel.groupSyncRead.txRxPacket()
            if dxl_comms_connect != dxl_class.COMM_SUCCESS:
                print("%s" % Dynamexel.packetHandler.getTxRxResult(dxl_comms_connect))
        else:
            # Add Dynamixel#x goal position value to the Syncread parameter storage
            dxl_getdata_result = Dynamexel.groupSyncRead.isAvailable(self.DXL_ID, self.ADDR_PRESENT_POSITION,
                                                                     self.LEN_PRESENT_POSITION)
            if not dxl_getdata_result:
                print("[ID:%03d] groupSyncRead getdata failed" % self.DXL_ID)
                quit()
            dxl_present_position = Dynamexel.groupSyncRead.getData(self.DXL_ID, self.ADDR_PRESENT_POSITION,
                                                                   self.LEN_PRESENT_POSITION)
            print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t" % (
                self.DXL_ID, self.dxl_target, dxl_present_position), end="")

            return abs(self.dxl_target - dxl_present_position) > self.DXL_MOVING_STATUS_THRESHOLD

    def _convert_val_to_goal_pos(self, pos):
        # convert goal position into byte array
        return [dxl_class.DXL_LOBYTE(dxl_class.DXL_LOWORD(pos)),
                dxl_class.DXL_HIBYTE(dxl_class.DXL_LOWORD(pos)),
                dxl_class.DXL_LOBYTE(dxl_class.DXL_HIWORD(pos)),
                dxl_class.DXL_HIBYTE(dxl_class.DXL_HIWORD(pos))]

if __name__ == "__main__":

    PROTOCOL_VERSION = 2.0
    DEVICENAME = '/dev/ttyUSB0'

    dxl_controllers = [Dynamexel(i, device_name=DEVICENAME, protocol=PROTOCOL_VERSION) for i in range(4)]

    # Use the actual port assigned to the U2D2.
    # .Windows: "COM*", Linux: "/dev/ttyUSB*", Mac: "/dev/tty.usbserial-*"TORQUE_DIS

    dxl_goal_position = range(0, 4096, 500)  # Goal position

    for index in range(4):
        dxl_controllers[index].init_dynamixels()
    pos_index = 0
    while 1:
        print("Press any key to continue! (or press ESC to quit!)")
        if getch() == chr(0x1b):
            for index in range(4):
                dxl_controllers[index].groupSyncRead.clearParam()
                dxl_controllers[index].disconnect_dynamixels()
            break

        for index in range(4):
            dxl_controllers[index].set_target_pos(target=dxl_goal_position[pos_index])
            dxl_controllers[index].sync_write_dynamixels()
        dxl_controllers[0].sync_write_dynamixels(send_tx=True)

        while 1:
            # Sync_read present position
            dxl_controllers[0].sync_read_dynamixels(receive_rx=True)

            # Check if groupsyncread data of Dynamixel#1 is available
            to_break = True
            for index in range(4):
                to_break &= dxl_controllers[index].sync_read_dynamixels()
            print()

            if not to_break:
                # Clear syncread parameter storage
                break

        pos_index += 1
        pos_index %= len(dxl_goal_position)

    dxl_controllers[0].close_port_handler()