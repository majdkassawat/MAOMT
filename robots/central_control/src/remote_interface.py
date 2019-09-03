#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Bool
from std_msgs.msg import Float64
from std_msgs.msg import UInt32
import time
import threading
import signal


cancel_signal = False
freq = 40
mode = "stop"
sync = False

# Defining Ros messages
stop_msg = Bool()
sync_msg = Bool()

def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True
signal.signal(signal.SIGINT, sigint_handler)


def remote_ch1_Callback(msg):
    x = 1+2
    # print("ch1: "+str(msg.data))


def remote_ch2_Callback(msg):
    x = 1+2
    # print("ch2: "+str(msg.data))


def remote_ch3_Callback(msg):
    x = 1+2
    # print("ch3: "+str(msg.data))


def remote_ch4_Callback(msg):
    x = 1+2
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

def remote_interface():
    # print("mode: "+ str(mode)+" sync: "+ str(sync))
    # Process stop signal
    if mode == "stop":
        stop_msg.data = True
    else:
        stop_msg.data = False
    # Process sync signal
    sync_msg.data = sync


    # Publish all messages
    stop_pub.publish(stop_msg)
    sync_pub.publish(sync_msg)
    # If cancel signal is not received (ctrl+c), then execute this function again respecting the frequency (here no need for ros sleep)
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, remote_interface).start()

remote_interface()
# r = rospy.Rate(freq)
# while not rospy.is_shutdown():
#     # interface()
#     r.sleep()
