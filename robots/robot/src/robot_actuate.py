#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D
from omnidirectional_driver import omnidirectional_driver
from dynamixel_driver_sync import DynamixelDriver


rotation_origin_msg = Pose2D()
speed_conversion_const = 0.111


def cmdTwistVelCallback(velocity_msg):
    global rotation_origin_msg
    wheel1_vel, wheel2_vel, wheel3_vel = omnidirectional_driver(
        velocity_msg, rotation_origin_msg)

    print(int(wheel1_vel/speed_conversion_const),
          int(wheel2_vel/speed_conversion_const), int(wheel3_vel/speed_conversion_const))
    dxl_driver.set_dxl_speed(1, int(wheel1_vel/speed_conversion_const))
    dxl_driver.set_dxl_speed(2, int(wheel2_vel/speed_conversion_const))
    dxl_driver.set_dxl_speed(3, int(wheel3_vel/speed_conversion_const))

    if wheel1_vel == 0:
        dxl_driver.set_dxl_torque_enable(1, False)
    if wheel2_vel == 0:
        dxl_driver.set_dxl_torque_enable(2, False)
    if wheel3_vel == 0:
        dxl_driver.set_dxl_torque_enable(3, False)
    dxl_driver.send_command()
    

    


def rotationOriginCallback(msg):
    global rotation_origin_msg
    rotation_origin_msg = msg


def cmdTractionLeftVelCallback(msg):
    traction_wheel_vel = msg.data
    if traction_wheel_vel == 0:
        dxl_driver.set_dxl_torque_enable(5, False)
    else:
        dxl_driver.set_dxl_torque_enable(5, True)
    dxl_driver.set_dxl_speed(5, int(traction_wheel_vel/speed_conversion_const))


def cmdTractionRightVelCallback(msg):
    traction_wheel_vel = msg.data
    if traction_wheel_vel == 0:
        dxl_driver.set_dxl_torque_enable(4, False)
    else:
        dxl_driver.set_dxl_torque_enable(4, True)
    dxl_driver.set_dxl_speed(4, int(traction_wheel_vel/speed_conversion_const))


dxl_driver = DynamixelDriver("/dev/ttyUSB0")
dxl_driver.open_port()
rospy.init_node('robot_actuate_node', log_level=rospy.DEBUG)


command_twist_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmdTwistVelCallback)
rotation_origin_sub = rospy.Subscriber(
    'rotation_origin', Pose2D, rotationOriginCallback)
command_traction_wheel_left_vel_sub = rospy.Subscriber(
    'traction_wheel_left_vel', Float64, cmdTractionLeftVelCallback)
command_traction_wheel_right_vel_sub = rospy.Subscriber(
    'traction_wheel_right_vel', Float64, cmdTractionRightVelCallback)


# r = rospy.Rate(100)
# while not rospy.is_shutdown():
#     r.sleep()

rospy.spin()
if rospy.is_shutdown():
    dxl_driver.set_dxl_torque_enable(1, False)
    dxl_driver.set_dxl_torque_enable(2, False)
    dxl_driver.set_dxl_torque_enable(3, False)
    dxl_driver.close_port()
