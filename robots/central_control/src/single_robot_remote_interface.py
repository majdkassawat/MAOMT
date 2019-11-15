#!/usr/bin/env python2.7
import rospy
# from std_msgs.msg import Bool
# from std_msgs.msg import Float64
from std_msgs.msg import UInt32
# from std_msgs.msg import Int64
import time
import threading
import signal
from scipy.interpolate import interp1d
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

cancel_signal = False
freq = 40
mode = "stop"
sync = False

min_x_vel = -100
max_x_vel = 100
min_y_vel = -100
max_y_vel = 100
min_angular_vel = -100
max_angular_vel = 100

ch1 = 0
ch2 = 0
ch3 = 0
ch4 = 0
# Defining Ros messages
cmd_vel_msg = Twist()


def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True


signal.signal(signal.SIGINT, sigint_handler)


def remote_ch1_Callback(msg):
    global ch1
    if msg.data >= 1000 and msg.data <= 2000:
        ch1 = msg.data
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch1: " + str(msg.data))
    # print("ch1: "+str(msg.data))


def remote_ch2_Callback(msg):
    global ch2
    if msg.data >= 1000 and msg.data <= 2000:
        ch2 = msg.data
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch2: " + str(msg.data))
    # print("ch2: "+str(msg.data))


def remote_ch3_Callback(msg):
    global ch3
    if msg.data >= 1000 and msg.data <= 2000:
        ch3 = msg.data
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch3: " + str(msg.data))
    # print("ch3: "+str(msg.data))


def remote_ch4_Callback(msg):
    global ch4
    if msg.data >= 1000 and msg.data <= 2000:
        ch4 = msg.data
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch4: " + str(msg.data))
    # print("ch4: "+str(msg.data))


def remote_ch5_Callback(msg):
    global mode
    if (msg.data > 0) and (msg.data <= 1200):
        mode = "stop"
    elif (msg.data > 1200) and (msg.data <= 1742):
        mode = "lock"
    elif (msg.data > 1742) and (msg.data <= 2500):
        mode = "control"
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch5: " + str(msg.data))


def remote_ch6_Callback(msg):
    global sync
    if (msg.data > 0) and (msg.data <= 1500):
        sync = False
    elif (msg.data > 1500) and (msg.data <= 2500):
        sync = True
    else:
        raise ValueError(
            "the value received from the transmitter is not valid on ch6: " + str(msg.data))


# sync_pub = rospy.Publisher(
#     '/sync', Bool, queue_size=10)
cmd_vel_pub = rospy.Publisher(
    '/cmd_vel', Twist, queue_size=10)
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


rospy.init_node('single_robot_remote_interface', log_level=rospy.INFO)


def remote_interface():
    # Process stop signal
    if mode == "stop":
        # stop_msg.data = True
        print("STOP")
        cmd_vel_msg.angular.z = 0
        cmd_vel_msg.linear.x = 0
        cmd_vel_msg.linear.y = 0
    # else:
    #     print("Running")
        # stop_msg.data = False
    # Process sync signal
    # sync_msg.data = sync
    # Process control mode
    if mode == "control":
        print("CONTROL")
        print("ch1:", ch1, "ch2:", ch2, "ch3:", ch3, "ch4:", ch4)
        # Generate cmd_vel from channels data
        x_vel_scale = (interp1d([1000, 2000], [min_x_vel, max_x_vel]))
        y_vel_scale = (interp1d([1000, 2000], [min_y_vel, max_y_vel]))
        angular_vel_scale = (
            interp1d([1000, 2000], [min_angular_vel, max_angular_vel]))
        x_vel = -1 * float(x_vel_scale(ch1))
        y_vel = float(y_vel_scale(ch2))
        angular_vel = float(angular_vel_scale(ch4))
        cmd_vel_msg.angular.z = angular_vel
        cmd_vel_msg.linear.x = x_vel
        cmd_vel_msg.linear.y = y_vel
    if mode == "lock":
        print("LOCKED")
    # Publish messages
    cmd_vel_pub.publish(cmd_vel_msg)
    # If cancel signal is not received (ctrl+c), then execute this function again respecting the frequency (here no need for ros sleep)
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, remote_interface).start()


remote_interface()
