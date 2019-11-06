#!/usr/bin/env python
# -*- coding: utf-8 -*-
import time
###################################################################
from dynamixel_driver import DynamixelDriver
dxl_driver = DynamixelDriver("/dev/ttyUSB0")
dxl_driver.open_port()
dxl_driver.set_dxl_led(2, True)
dxl_driver.set_dxl_torque_enable(2, True)
dxl_driver.set_dxl_speed(2, 500)
time.sleep(5)
dxl_driver.set_dxl_speed(2, 0)
dxl_driver.set_dxl_torque_enable(2, False)
dxl_driver.set_dxl_led(2, False)
dxl_driver.close_port()
