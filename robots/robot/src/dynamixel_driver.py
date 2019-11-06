#!/usr/bin/env python
# -*- coding: utf-8 -*-

###################################################################
# Dynamixel A12 driver
# - Control LED on, off
# - Control Torque on, off
# - Control Speed 0 - 1023
# - Control Port open, close
#

from dynamixel_sdk import *                    # Uses Dynamixel SDK library
import os

if os.name == 'nt':
    import msvcrt

    def getch():
        return msvcrt.getch().decode()
else:
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)

    def getch():
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch


# Control table address
ADDR_A12_TORQUE_ENABLE = 24
ADDR_A12_LED = 25
ADDR_A12_MOVING_SPEED = 32
# Protocol version
# See which protocol version is used in the Dynamixel
PROTOCOL_VERSION = 1.0
# Default setting
BAUDRATE = 1000000


class DynamixelDriver:

    def __init__(self, port_name):
        # Initialize PortHandler instance
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.portHandler = PortHandler(port_name)

        # Initialize PacketHandler instance
        # Set the protocol version
        # Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

    def open_port(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port")
        else:
            print("Failed to open the port")
            print("Press any key to terminate...")
            getch()
            quit()

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate")
        else:
            print("Failed to change the baudrate")
            print("Press any key to terminate...")
            getch()
            quit()

    def close_port(self):
        # Close port
        self.portHandler.closePort()

    def set_dxl_led(self, dxl_id, led_command):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, ADDR_A12_LED, led_command)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("led set")

    def set_dxl_speed(self, dxl_id, speed):
        dxl_comm_result, dxl_error = self.packetHandler.write2ByteTxRx(
            self.portHandler, dxl_id, ADDR_A12_MOVING_SPEED, speed)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Speed set")

    def set_dxl_torque_enable(self, dxl_id, torque_enable):
        if torque_enable == True:
            torque_enable = 1
        else:
            torque_enable = 0
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, ADDR_A12_TORQUE_ENABLE, torque_enable)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        else:
            print("Torque enable set")


# Enable Dynamixel Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Dynamixel has been successfully connected")

# # Set torque limit
# dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_TORQUE_LIMIT, TORQUE_LIMIT)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print(dxl_error)
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Torque limit has been set")

# dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_LED, 1)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print(dxl_error)
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("LED has been set")

# while 1:
# dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_MOVING_SPEED, SPEED)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))
# else:
#     print("Speed sent")

# while 1:
# print("Press any key to continue! (or press ESC to quit!)")
# if getch() == chr(0x1b):
#     break

# # Write goal position
# dxl_comm_result, dxl_error = packetHandler.write4ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_GOAL_POSITION, dxl_goal_position[index])
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

#  while 1:
#      # Read present position
#      dxl_present_position, dxl_comm_result, dxl_error = packetHandler.read4ByteTxRx(
#          portHandler, DXL_ID, ADDR_MX_PRESENT_POSITION)
#      if dxl_comm_result != COMM_SUCCESS:
#          print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
#      elif dxl_error != 0:
#          print("%s" % packetHandler.getRxPacketError(dxl_error))

#      print("[ID:%03d] GoalPos:%03d  PresPos:%03d" %
#            (DXL_ID, dxl_goal_position[index], dxl_present_position))

#      if not abs(dxl_goal_position[index] - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD:
#          break

# # Change goal position
# if index == 0:
#     index = 1
# else:
#     index = 0


# Disable Dynamixel Torque
# dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(
#     portHandler, DXL_ID, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE)
# if dxl_comm_result != COMM_SUCCESS:
#     print("%s" % packetHandler.getTxRxResult(dxl_comm_result))
# elif dxl_error != 0:
#     print("%s" % packetHandler.getRxPacketError(dxl_error))

# # Close port
# portHandler.closePort()
