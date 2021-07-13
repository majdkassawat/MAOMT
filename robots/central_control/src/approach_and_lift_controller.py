#!/usr/bin/env python2.7
import rospy
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Int64
from std_msgs.msg import Int16
from std_msgs.msg import Bool
import math
import tf
import time
import threading
import signal
from collections import Iterable
import json
import os
import pprint


cancel_signal = False
stop_signal = False
remote_control_activated = False
min_pressure_delta = 0
robot_orientation = 0
height = 0
# Frequency, this value is overwritten by the config file
freq = 40
# Height offset to be set in calibrate
height_offset = 0
# Dictionary visible markers
refrences_dict = {}
refrences_dict_lock = False
# Current_velocities_global
current_vel_x = 0
current_vel_y = 0
current_vel_angular = 0
current_vel_t_left = 0
current_vel_t_right = 0
# Old_velocities_global
old_vel_x = 0
old_vel_y = 0
old_vel_angular = 0
old_vel_t_left = 0
old_vel_t_right = 0
# Current_target_point_index
crnt_trgt_pnt_idx = 0
# velocity message
cmd_vel_msg = Twist()
t_left_msg = Float64()
t_right_msg = Float64()
# position message
rotation_origin_msg = Pose2D()
rotation_origin_msg.theta = 60
# point message
current_point_msg = Int16()
# timers list
time_init_dict = {}
trajectory_plan = {}

script_dir = os.path.dirname(os.path.realpath(__file__))
parent_dir = os.path.abspath(os.path.join(script_dir, os.pardir))
config_dir = parent_dir + "/config/"
# print(config_dir)
with open(config_dir + 'trj_remote.json') as json_file:
    config_file = json.load(json_file)
    trajectory_plan = config_file["trajectory"]
    remote_control_activated = config_file["remote_control_activated"]
    freq = config_file["freq"]


def ramp(beginning_value, ending_value):
    return_value = beginning_value
    slope_factor = 100
    if abs(beginning_value - ending_value) <= slope_factor:
        return ending_value
    elif beginning_value > ending_value:
        return_value = beginning_value - slope_factor
    elif beginning_value < ending_value:
        return_value = beginning_value + slope_factor

    return return_value


def stop():
    global current_vel_x, current_vel_y, current_vel_angular, current_vel_t_left, current_vel_t_right
    current_vel_x = ramp(current_vel_x, 0)
    current_vel_y = ramp(current_vel_y, 0)
    current_vel_t_left = 0
    current_vel_t_right = 0
    current_vel_angular = ramp(current_vel_angular, 0)


def process_correction(correction):
    global current_vel_x, current_vel_y, current_vel_z, current_vel_angular, current_vel_t_left, current_vel_t_right
    global refrences_dict, rotation_origin, remote_control_activated, robot_orientation, height, height_offset
    rotation_origin_msg.x = 0
    rotation_origin_msg.y = 0
    if correction["type"] == "single":
        error = refrences_dict[correction["reference"]
                               ]["value"] - correction["target"]
        print("reference ", refrences_dict[correction["reference"]]["value"])
        print("target ", correction["target"])

        k = correction["controller_parameters"]["k"]
        bias = correction["controller_parameters"]["bias"]
        error_tolerance = correction["error_tolerance"]
        target_vel = (error) * k + (error)/abs(error) * bias

        if correction["reference"] == "401_x":
            # correct z
            if(abs(error) > error_tolerance):
                # rospy.loginfo("correcting 401_x, error ="+str(error))
                # stop()
                current_vel_y = ramp(current_vel_y, target_vel)
                correction["finished"] = False
            else:
                stop()
                correction["finished"] = True
        elif correction["reference"] == "401_z":
            # correct x
            if(abs(error) > error_tolerance):
                # rospy.loginfo("correcting 401_z, error ="+str(error))
                # stop()
                current_vel_x = ramp(current_vel_x, target_vel)
                correction["finished"] = False
            else:
                stop()
                correction["finished"] = True
        elif correction["reference"] == "401_pitch":
            # correct pitch
            if(abs(error) > error_tolerance):
                # rospy.loginfo("correcting 401_pitch, error ="+str(error))

                rotation_origin_msg.x = 0
                rotation_origin_msg.y = 0
                # stop()
                current_vel_angular = ramp(current_vel_angular, target_vel)
                correction["finished"] = False
            else:
                stop()
                correction["finished"] = True
        elif correction["reference"] == "401_yaw":
            # correct yaw
            if(abs(error) > error_tolerance):
                # rospy.loginfo("correcting 401_yaw, error ="+str(error))
                # stop()
                current_vel_t_left = ramp(current_vel_t_left, target_vel)
                current_vel_t_right = ramp(current_vel_t_right, target_vel)
                correction["finished"] = False
            else:
                stop()
                correction["finished"] = True
        elif correction["reference"] == "401_y":
            if remote_control_activated == True:
                error = refrences_dict[correction["reference"]
                                       ]["value"]-height
                k = correction["controller_parameters"]["k"]
                bias = correction["controller_parameters"]["bias"]
                error_tolerance = correction["error_tolerance"]
                target_vel = (error) * k + (error)/abs(error) * bias
            # correct height
            elif(abs(error) > error_tolerance):
                # rospy.loginfo("correcting 401_y, error ="+str(error))
                # stop()
                current_vel_t_left = ramp(current_vel_t_left, target_vel)
                current_vel_t_right = ramp(
                    current_vel_t_right, -1 * target_vel)
                correction["finished"] = False
            else:
                stop()
                correction["finished"] = True
    elif correction["type"] == "secuencial":
        for i in range(0, len(correction)-2):
            if correction[str(i)]["finished"] == False:
                process_correction(correction[str(i)])
                break
            else:
                continue
        if correction[str(len(correction)-3)]["finished"] == True:
            correction["finished"] = True
    elif correction["type"] == "parallel":
        # defining sum variables
        sum_vel_x = 0
        sum_vel_y = 0
        sum_vel_angular = 0
        sum_vel_t_left = 0
        sum_vel_t_right = 0
        correction["finished"] = True
        for i in range(0, len(correction) - 2):
            stop()  # This is to zero the values before we calculate new correction, it does not stop the robot.
            process_correction(correction[str(i)])
            # agregating the values
            sum_vel_x = sum_vel_x + current_vel_x
            sum_vel_y = sum_vel_y + current_vel_y
            sum_vel_angular = sum_vel_angular + current_vel_angular
            sum_vel_t_left = sum_vel_t_left + current_vel_t_left
            sum_vel_t_right = sum_vel_t_right + current_vel_t_right
            if correction[str(i)]["finished"] == False:
                correction["finished"] = False
        # reassigning the summed values to current values to be outputted
        current_vel_x = sum_vel_x
        current_vel_y = sum_vel_y
        current_vel_angular = sum_vel_angular
        current_vel_t_left = sum_vel_t_left
        current_vel_t_right = sum_vel_t_right
    elif correction["type"] == "semi-secuencial":
        for i in range(0, len(correction)-2):
            if correction[str(i)]["finished"] == False:
                process_correction(correction[str(i)])
                break
            else:
                correction[str(i)]["finished"] = False
                continue
        if correction[str(len(correction)-3)]["finished"] == True:
            correction["finished"] = True
    elif correction["type"] == "force_coupled":
        k_turn = correction["controller_parameters"]["k_turn"]
        bias_turn = correction["controller_parameters"]["bias_turn"]
        k_linear = correction["controller_parameters"]["k_linear"]
        bias_linear = correction["controller_parameters"]["bias_linear"]
        fl = refrences_dict[correction["reference"]["left"]]["value"]
        fr = refrences_dict[correction["reference"]["right"]]["value"]
        target = correction["target"]
        target_plus_delta = target
        if remote_control_activated == True:
            if robot_orientation == 180 and min_pressure_delta > 0:
                target_plus_delta = target + abs(min_pressure_delta)
            elif robot_orientation == 180 and min_pressure_delta < 0:
                target_plus_delta = target - abs(min_pressure_delta)
            elif robot_orientation == 0 and min_pressure_delta < 0:
                target_plus_delta = target + abs(min_pressure_delta)
            elif robot_orientation == 0 and min_pressure_delta > 0:
                target_plus_delta = target - abs(min_pressure_delta)

        difference_error = fl - fr
        # Adding delta to target in case of remote control activated (depends on orientation)
        if abs(difference_error) > correction["difference_pressure_tolerance"]:
            print("correcting difference")

            target_vel = (difference_error) * k_turn + \
                (float(difference_error)/abs(difference_error)) * bias_turn
            current_vel_x = ramp(current_vel_x, 0)
            # current_vel_x = 0
            current_vel_angular = ramp(current_vel_angular, target_vel)
            # if current_vel_angular < 0 :
            #     rotation_origin_msg.x = -12
            #     rotation_origin_msg.y = 0
            # else:
            #     rotation_origin_msg.x = 12
            #     rotation_origin_msg.y = 0
            # current_vel_angular = target_vel
        elif (fl < target_plus_delta and fr < target_plus_delta) or (fl > target_plus_delta and fr > target_plus_delta):
            # go forward or backward
            print("correcting linear")
            average_error = target_plus_delta - 0.5 * (fl+fr)
            target_vel = (average_error) * k_linear + \
                (float(average_error)/abs(average_error)) * bias_linear
            current_vel_x = ramp(current_vel_x, target_vel)
            # current_vel_x = target_vel
            current_vel_angular = ramp(current_vel_angular, 0)
            # current_vel_angular = 0
       
        else:
            print("forces are stable .. nothing to correct")
            # stop
            current_vel_x = ramp(current_vel_x, 0)
            # current_vel_x = 0
            current_vel_angular = ramp(current_vel_angular, 0)
            # current_vel_angular = 0
            correction["finished"] = True
    elif correction["type"] == "single_boolean":
        if correction["reference"] == "sync":
            target = correction["target"]
            # print("target:" + str(target) + "remote:" +
            #       str(refrences_dict[correction["reference"]]["value"]))
            if target == refrences_dict[correction["reference"]]["value"]:
                correction["finished"] = True
            else:
                correction["finished"] = False
    elif correction["type"] == "calibrate_y":
        height_offset = refrences_dict["401_y"]["value"]
        correction["finished"] = True
    elif correction["type"] == "stop":
        stop()
        print("stop point is called")
        correction["finished"] = True
    elif correction["type"] == "wait":
        if correction["id"] in time_init_dict:
            current_time = time.time()  # in seconds
            elapsed_time = current_time - time_init_dict[correction["id"]]
            if elapsed_time >= correction["target"]:
                correction["finished"] = True
            else:
                print("waiting")
                correction["finished"] = False
        else:
            current_time = time.time()  # in seconds
            time_init_dict[correction["id"]] = current_time
            correction["finished"] = False


    return correction["finished"]


def Markers_list_Callback(msg):
    global refrences_dict

    if refrences_dict_lock == False:
        for id, refrence in refrences_dict.items():
            if refrence["type"] == "marker":
                marker_id = id.split('_')[0]

                if int(marker_id) not in msg.data:
                    del refrences_dict[id]


def MarkersCallback(msg):
    global refrences_dict

    for marker in msg.markers:
        orientation_q = marker.pose.pose.orientation
        orientation_list = [
            orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
            orientation_list)

        # rospy.loginfo ("yaw:"+str(yaw)+" pitch:"+str(pitch)+" roll:"+str(roll))
        x = marker.pose.pose.position.x
        y = marker.pose.pose.position.y
        z = marker.pose.pose.position.z
        # if yaw < 0:
        #     yaw = math.pi * 2 - abs(yaw)
        refrences_dict[str(marker.id)+"_x"] = {"type": "marker", "value": x}
        refrences_dict[str(
            marker.id)+"_y"] = {"type": "marker", "value": y - height_offset}
        refrences_dict[str(marker.id)+"_z"] = {"type": "marker", "value": z}
        refrences_dict[str(marker.id) +
                       "_roll"] = {"type": "marker", "value": roll}
        refrences_dict[str(marker.id) +
                       "_pitch"] = {"type": "marker", "value": pitch}
        refrences_dict[str(marker.id) +
                       "_yaw"] = {"type": "marker", "value": yaw}

        # print("yaw:"+str(yaw)+" pitch:"+str(pitch)," roll:"+str(roll))


def force_sensor_left_Callback(msg):
    global refrences_dict
    refrences_dict["fsl"] = {"type": "force_sensor"}
    refrences_dict["fsl"]["value"] = msg.data
    # print(refrences_dict["fsl"]["value"])


def force_sensor_right_Callback(msg):
    global refrences_dict
    refrences_dict["fsr"] = {"type": "force_sensor"}
    refrences_dict["fsr"]["value"] = msg.data
    # print(refrences_dict["fsr"]["value"])


def stop_signal_Callback(msg):
    global stop_signal
    # stop_signal = msg.data
    stop_signal = False # This is for debugging


def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True


def sync_Callback(msg):
    global refrences_dict
    refrences_dict["sync"] = {"type": "synchronization"}
    refrences_dict["sync"]["value"] = msg.data


def height_target_Callback(msg):
    global height
    height = msg.data
    # print(msg.data)


def min_pressure_delta_Callback(msg):
    global min_pressure_delta
    min_pressure_delta = msg.data
    # print(msg.data)


def orientation_Callback(msg):
    global robot_orientation
    robot_orientation = msg.data
    # print(msg.data)


signal.signal(signal.SIGINT, sigint_handler)

# Preparing Ros nodes and topics
rospy.init_node('approach_and_lift_controller', log_level=rospy.INFO)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
rotation_origin_pub = rospy.Publisher('rotation_origin', Pose2D, queue_size=10)
t_left_vel_pub = rospy.Publisher(
    'traction_wheel_left_vel', Float64, queue_size=10)
t_right_vel_pub = rospy.Publisher(
    'traction_wheel_right_vel', Float64, queue_size=10)
current_point_pub = rospy.Publisher(
    'current_point', Int16, queue_size=10)
sub = rospy.Subscriber('aruco_marker_publisher/markers',
                       MarkerArray, MarkersCallback)
sub_markers_list = rospy.Subscriber(
    'aruco_marker_publisher/markers_list', UInt32MultiArray, Markers_list_Callback)
sub_force_sensor_left = rospy.Subscriber(
    'force_sensor_left', Int16, force_sensor_left_Callback)
sub_force_sensor_right = rospy.Subscriber(
    'force_sensor_right', Int16, force_sensor_right_Callback)
sub_stop_signal = rospy.Subscriber(
    '/stop_signal', Bool, stop_signal_Callback)
sub_sync = rospy.Subscriber(
    '/sync', Bool, sync_Callback)
sub_height_target = rospy.Subscriber(
    '/height_target', Float64, height_target_Callback)
sub_min_pressure = rospy.Subscriber(
    '/min_pressure_delta', Int64, min_pressure_delta_Callback)
sub_orientation = rospy.Subscriber(
    'orientation', Float64, orientation_Callback)



def controller():
    global freq, refrences_dict, crnt_trgt_pnt_idx, trajectory_plan, current_vel_x, current_vel_y, current_vel_angular, current_vel_t_left, current_vel_t_right, old_vel_angular, old_vel_t_left, old_vel_t_right, old_vel_x, old_vel_y, refrences_dict_lock
    
    refrences_dict_lock = True
    stop() # This is to zero all velocities before starting calculations
    current_point_msg.data = crnt_trgt_pnt_idx
    if crnt_trgt_pnt_idx < len(trajectory_plan):
        current_point = trajectory_plan[str(crnt_trgt_pnt_idx)]
        if current_point["reached"] == True:  # point has already been reached
            crnt_trgt_pnt_idx = crnt_trgt_pnt_idx + 1  # move to next point
            rospy.loginfo("reached point"+str(crnt_trgt_pnt_idx))
        else:  # point has not been processed yet
            rospy.loginfo("processing point"+str(crnt_trgt_pnt_idx))
            # check that all references for this point are available
            required_reference_list = []

            def fill_required_reference_list(element):
                if isinstance(element, dict):
                    if "reference" in element:
                        if isinstance(element["reference"], dict):
                            for ref in element["reference"]:
                                required_reference_list.append(
                                    element["reference"][ref])
                        else:
                            required_reference_list.append(
                                element["reference"])
                    else:
                        for sub_element_key in element:
                            fill_required_reference_list(
                                element[sub_element_key])
            fill_required_reference_list(current_point["correction"])

            def check_all_references_available():
                for required_ref in required_reference_list:
                    if (required_ref not in refrences_dict) and ((required_ref != "fsl")or(required_ref != "fsr")):
                        return False
                return True
            # rospy.loginfo("required_references:"+" ".join(required_reference_list))
            if check_all_references_available() == True:
                # rospy.loginfo("processing point: " + str(crnt_trgt_pnt_idx) + ", reference found")
                # print(refrences_dict)
                current_point["reached"] = process_correction(
                    current_point["correction"])
                # print("left:", refrences_dict["fsl"]["value"],
                #       " right:", refrences_dict["fsr"]["value"])
            else:  # reference marker not found, stop
                # rospy.loginfo("processing point: " + str(crnt_trgt_pnt_idx) + ", reference NOT found")
                stop()
    else:  # reached the end of the trajectory
        stop()

    refrences_dict_lock = False
    # print("before updating speeds")
    print("Angular ", current_vel_angular, " x ",
          current_vel_x, " y", current_vel_y, " t_right ", current_vel_t_right, " t_left ", current_vel_t_left)

    # rospy.loginfo(refrences_dict)
    # prepare actuating messages to be published
    if True:
        old_vel_angular = current_vel_angular
        old_vel_x = current_vel_x
        old_vel_y = current_vel_y
        old_vel_t_right = current_vel_t_right
        old_vel_t_left = current_vel_t_left

        if stop_signal == True:
            cmd_vel_msg.linear.x = 0
            cmd_vel_msg.linear.y = 0
            cmd_vel_msg.angular.z = 0
            t_left_msg.data = 0
            t_right_msg.data = 0
        else:
            cmd_vel_msg.linear.x = current_vel_x
            cmd_vel_msg.linear.y = current_vel_y
            cmd_vel_msg.angular.z = current_vel_angular
            t_left_msg.data = current_vel_t_left
            t_right_msg.data = current_vel_t_right
        cmd_vel_pub.publish(cmd_vel_msg)
        t_left_vel_pub.publish(t_left_msg)
        t_right_vel_pub.publish(t_right_msg)
        rotation_origin_pub.publish(rotation_origin_msg)
        current_point_pub.publish(current_point_msg)
        print("time:", time.time())
    # set the timer to fire up the same function in the specifc time period
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, controller).start()
    else:
        stop()
        cmd_vel_pub.publish(cmd_vel_msg)
        t_left_vel_pub.publish(t_left_msg)
        t_right_vel_pub.publish(t_right_msg)

controller()
r = rospy.Rate(freq)
while not rospy.is_shutdown():
    r.sleep()
