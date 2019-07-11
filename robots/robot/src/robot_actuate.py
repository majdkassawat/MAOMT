#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from omnidirectional_driver import omnidirectional_driver

traction_wheel_left_controller_msg = Float64()
traction_wheel_right_controller_msg = Float64()
wheel1_controller_msg = Float64()
wheel2_controller_msg = Float64()
wheel3_controller_msg = Float64()


def cmdTwistVelCallback(msg):
    wheel1_vel,wheel2_vel,wheel3_vel = omnidirectional_driver(msg)
    wheel1_controller_msg.data=wheel1_vel
    wheel2_controller_msg.data=wheel2_vel
    wheel3_controller_msg.data=wheel3_vel
    wheel1_controller_pub.publish(wheel1_controller_msg)
    wheel2_controller_pub.publish(wheel2_controller_msg)
    wheel3_controller_pub.publish(wheel3_controller_msg)
    # print("INPUT:",msg.angular.z,msg.linear.x,msg.linear.y)
    # print("OUTPUT:",wheel1_vel,wheel2_vel,wheel3_vel)
    


def cmdTractionLeftVelCallback(msg):
    traction_wheel_left_controller_msg.data=msg.data
    traction_wheel_left_controller_pub.publish(traction_wheel_left_controller_msg)


def cmdTractionRightVelCallback(msg):
    traction_wheel_right_controller_msg.data=msg.data
    traction_wheel_right_controller_pub.publish(traction_wheel_right_controller_msg)



rospy.init_node('robot_actuate_node', log_level=rospy.DEBUG)

traction_wheel_left_controller_pub = rospy.Publisher('traction_wheel_left_controller/command', Float64, queue_size=1)
traction_wheel_right_controller_pub = rospy.Publisher('traction_wheel_right_controller/command', Float64, queue_size=1)
wheel1_controller_pub = rospy.Publisher('wheel1_controller/command', Float64, queue_size=1)
wheel2_controller_pub = rospy.Publisher('wheel2_controller/command', Float64, queue_size=1)
wheel3_controller_pub = rospy.Publisher('wheel3_controller/command', Float64, queue_size=1)

command_twist_vel_sub = rospy.Subscriber('cmd_vel', Twist, cmdTwistVelCallback)
command_traction_wheel_left_vel_sub = rospy.Subscriber('traction_wheel_left_vel', Float64, cmdTractionLeftVelCallback)
command_traction_wheel_right_vel_sub = rospy.Subscriber('traction_wheel_right_vel', Float64, cmdTractionRightVelCallback)


r = rospy.Rate(100)
while not rospy.is_shutdown():
    r.sleep()