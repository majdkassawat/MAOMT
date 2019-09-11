#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import UInt32
from std_msgs.msg import Int64
import time
import threading
import signal
from scipy.interpolate import interp1d

cancel_signal = False
freq = 40
mode = "stop"
sync = False
height = 0
min_height = 0
max_height = 0.07
min_delta_pressure = -150
max_delta_pressure = 150

ch1 = 0
ch2 = 0
ch3 = 0
ch4 = 0
# Defining Ros messages
stop_msg = Bool()
sync_msg = Bool()
height_msg = Float64()
min_pressure_delta_msg = Int64()


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


sync_pub = rospy.Publisher(
    '/sync', Bool, queue_size=10)
stop_pub = rospy.Publisher(
    '/stop_signal', Bool, queue_size=10)
height_pub = rospy.Publisher(
    '/height_target', Float64, queue_size=10)
min_pressure_delta_pub = rospy.Publisher(
    '/min_pressure_delta', Int64, queue_size=10)
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


rospy.init_node('remote_interface', log_level=rospy.INFO)


def remote_interface():
    # Process stop signal
    if mode == "stop":
        stop_msg.data = True
        print ("STOP")
    else:
        stop_msg.data = False
    # Process sync signal
    sync_msg.data = sync
    # Process control mode
    if mode == "control":
        print("CONTROL")
        # Process height
        height_from_remote = (interp1d([1000, 2000], [min_height, max_height]))
        height = float(height_from_remote(ch3))
        height_msg.data = height
        # Process pressure delta
        min_pressure_delta_from_remote = (
            interp1d([1000, 2000], [min_delta_pressure, max_delta_pressure]))
        min_pressure_delta = int(min_pressure_delta_from_remote(ch2))
        min_pressure_delta_msg.data = min_pressure_delta
    if mode == "lock":
        print ("LOCKED")
    # Publish all messages
    stop_pub.publish(stop_msg)
    sync_pub.publish(sync_msg)
    height_pub.publish(height_msg)
    min_pressure_delta_pub.publish(min_pressure_delta_msg)
    # If cancel signal is not received (ctrl+c), then execute this function again respecting the frequency (here no need for ros sleep)
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, remote_interface).start()


remote_interface()

