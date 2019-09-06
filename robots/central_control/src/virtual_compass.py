#!/usr/bin/env python2.7
import rospy
from std_msgs.msg import Float64
import time
import threading
import signal


cancel_signal = False
freq = 40

robot1_orientation_msg = Float64()
robot2_orientation_msg = Float64()

robot1_orientation_msg.data = 0
robot2_orientation_msg.data = 180

def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True


signal.signal(signal.SIGINT, sigint_handler)

robot1_pub = rospy.Publisher(
    '/robot1/orientation', Float64, queue_size=10)
robot2_pub = rospy.Publisher(
    '/robot2/orientation', Float64, queue_size=10)

rospy.init_node('virtual_compass', log_level=rospy.INFO)


def main_publisher():

   robot1_pub.publish(robot1_orientation_msg)
   robot2_pub.publish(robot2_orientation_msg)
   # If cancel signal is not received (ctrl+c), then execute this function again respecting the frequency (here no need for ros sleep)
   if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, main_publisher).start()


main_publisher()
