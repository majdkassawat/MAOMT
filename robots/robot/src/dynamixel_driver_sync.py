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

        # Initialize GroupSyncWrite instance
        self.speedGroupSyncWrite = GroupSyncWrite(
            self.portHandler, self.packetHandler, ADDR_A12_MOVING_SPEED, 2)

        # Allocate goal position value into byte array
        param_speed = [DXL_LOBYTE(DXL_LOWORD(0)), DXL_HIBYTE(DXL_LOWORD(
            0))]

        # Adding Dynamixels params to speed group synch write
        dxl_addparam_result = self.speedGroupSyncWrite.addParam(
            1, param_speed)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % 1)
            quit()
        dxl_addparam_result = self.speedGroupSyncWrite.addParam(
            2, param_speed)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % 2)
            quit()
        dxl_addparam_result = self.speedGroupSyncWrite.addParam(
            3, param_speed)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % 3)
            quit()
        dxl_addparam_result = self.speedGroupSyncWrite.addParam(
            4, param_speed)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % 4)
            quit()
        dxl_addparam_result = self.speedGroupSyncWrite.addParam(
            5, param_speed)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncWrite addparam failed" % 5)
            quit()

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

    def send_command(self):
        # print("sending command")
        # Syncwrite goal position
        dxl_comm_result = self.speedGroupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        # print("command sent")

    def set_dxl_led(self, dxl_id, led_command):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, dxl_id, ADDR_A12_LED, led_command)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("led set")

    def set_dxl_speed(self, dxl_id, speed):
        if speed > 1023 or speed < -1023:
            print("speed value unaccepted")
            return
        if speed < 0:
            # set negative direction
            speed = abs(speed) + 1024
        # Allocate goal position value into byte array
        param_speed = [DXL_LOBYTE(DXL_LOWORD(speed)), DXL_HIBYTE(DXL_LOWORD(
            speed))]
        self.speedGroupSyncWrite.changeParam(dxl_id, param_speed)

    def set_dxl_torque_enable(self, dxl_id, torque_enable):
        if torque_enable == True:
            torque_enable = 1
        else:
            torque_enable = 0
        self.packetHandler.write1ByteTxOnly(
            self.portHandler, dxl_id, ADDR_A12_TORQUE_ENABLE, torque_enable)
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        # elif dxl_error != 0:
        #     print("%s" % self.packetHandler.getRxPacketError(dxl_error))
        # else:
        #     print("Torque enable set")
