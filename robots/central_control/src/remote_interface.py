#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import UInt32


freq = 40


def remote_ch1_Callback(msg):
    print("ch1: "+str(msg.data))


def remote_ch2_Callback(msg):
    print("ch2: "+str(msg.data))


def remote_ch3_Callback(msg):
    print("ch3: "+str(msg.data))


def remote_ch4_Callback(msg):
    print("ch4: "+str(msg.data))


def remote_ch5_Callback(msg):
    print("ch5: "+str(msg.data))


def remote_ch6_Callback(msg):
    print("ch6: "+str(msg.data))


t_left_vel_pub = rospy.Publisher(
    'traction_wheel_left_vel', Float64, queue_size=10)
sub_remote_ch1 = rospy.Subscriber(
    'remote_transmitter/ch1', UInt32, remote_ch1_Callback)
sub_remote_ch2 = rospy.Subscriber(
    'remote_transmitter/ch2', UInt32, remote_ch2_Callback)
sub_remote_ch3 = rospy.Subscriber(
    'remote_transmitter/ch3', UInt32, remote_ch3_Callback)
sub_remote_ch4 = rospy.Subscriber(
    'remote_transmitter/ch4', UInt32, remote_ch4_Callback)
sub_remote_ch5 = rospy.Subscriber(
    'remote_transmitter/ch5', UInt32, remote_ch5_Callback)
sub_remote_ch6 = rospy.Subscriber(
    'remote_transmitter/ch6', UInt32, remote_ch6_Callback)


# def interface():
# t_left_vel_pub.publish(t_left_msg)

rospy.init_node('remote_interface', log_level=rospy.INFO)
r = rospy.Rate(freq)
while not rospy.is_shutdown():
    # interface()
    r.sleep()
