#!/usr/bin/env python2.7
import rospy
from aruco_msgs.msg import MarkerArray
from std_msgs.msg import UInt32MultiArray
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from std_msgs.msg import Int64
from std_msgs.msg import Bool
import rospy
import math
import tf
import time
import threading
import signal
from collections import Iterable

cancel_signal = False
stop_signal = False

# Frequency
freq = 40
# Dictionary visible markers
refrences_dict = {}
refrences_dict_lock = False
# Current_velocities_global
current_vel_x = 0
current_vel_y = 0
current_vel_angular = 0
current_vel_t_left = 0
current_vel_t_right = 0
# Current_target_point_index
crnt_trgt_pnt_idx = 0
# velocity message
cmd_vel_msg = Twist()
t_left_msg = Float64()
t_right_msg = Float64()


filtering_value = 300


# modes of correction are :parallel, secuencial, single, semi-secuencial
trajectory_plan = {"0": {}}  # trajectory
# definition of the first point
# trajectory_plan["0"]["reached"] = False
# trajectory_plan["0"]["correction"] = {
#     "type": "semi-secuencial", "finished": False, "0": {}, "1": {},"2":{}}
# trajectory_plan["0"]["correction"]["2"]["type"] = "single"
# trajectory_plan["0"]["correction"]["2"]["reference"] = "401_x"
# trajectory_plan["0"]["correction"]["2"]["error_tolerance"] = 0.005
# trajectory_plan["0"]["correction"]["2"]["controller_parameters"] = {
#     "k": -8, "bias": -2}
# trajectory_plan["0"]["correction"]["2"]["target"] = 0
# trajectory_plan["0"]["correction"]["2"]["finished"] = False
# trajectory_plan["0"]["correction"]["0"]["type"] = "single"
# trajectory_plan["0"]["correction"]["0"]["reference"] = "401_pitch"
# trajectory_plan["0"]["correction"]["0"]["error_tolerance"] = 0.05
# trajectory_plan["0"]["correction"]["0"]["controller_parameters"] = {
#     "k": 5, "bias": 2}
# trajectory_plan["0"]["correction"]["0"]["target"] = 0
# trajectory_plan["0"]["correction"]["0"]["finished"] = False
# trajectory_plan["0"]["correction"]["1"]["type"] = "single"
# trajectory_plan["0"]["correction"]["1"]["reference"] = "401_z"
# trajectory_plan["0"]["correction"]["1"]["error_tolerance"] = 0.005
# trajectory_plan["0"]["correction"]["1"]["controller_parameters"] = {
#     "k": 5, "bias": 2}
# trajectory_plan["0"]["correction"]["1"]["target"] = 0.2
# trajectory_plan["0"]["correction"]["1"]["finished"] = False


trajectory_plan["0"]["reached"] = False
trajectory_plan["0"]["correction"] = {
    "type": "parallel", "finished": False, "0": {}, "1": {}, "2": {}}


trajectory_plan["0"]["correction"]["0"]["type"] = "force_coupled"
trajectory_plan["0"]["correction"]["0"]["reference"] = {
    "left": "fsl", "right": "fsr"}
trajectory_plan["0"]["correction"]["0"]["total_pressure_tolerance"] = 0.00005
trajectory_plan["0"]["correction"]["0"]["difference_pressure_tolerance"] = 20
trajectory_plan["0"]["correction"]["0"]["controller_parameters"] = {
    "k_turn": -0.03, "bias_turn": 0, "k_linear": 0.03, "bias_linear": 0}
trajectory_plan["0"]["correction"]["0"]["target"] = 100
trajectory_plan["0"]["correction"]["0"]["finished"] = False

trajectory_plan["0"]["correction"]["1"]["type"] = "single"
trajectory_plan["0"]["correction"]["1"]["reference"] = "401_yaw"
trajectory_plan["0"]["correction"]["1"]["error_tolerance"] = 0.0000005
trajectory_plan["0"]["correction"]["1"]["controller_parameters"] = {
    "k": 1, "bias": 1}
trajectory_plan["0"]["correction"]["1"]["target"] = 3.14
trajectory_plan["0"]["correction"]["1"]["finished"] = False

trajectory_plan["0"]["correction"]["2"]["type"] = "single"
trajectory_plan["0"]["correction"]["2"]["reference"] = "401_y"
trajectory_plan["0"]["correction"]["2"]["error_tolerance"] = 0.0000005
trajectory_plan["0"]["correction"]["2"]["controller_parameters"] = {
    "k": 300, "bias": 0}
trajectory_plan["0"]["correction"]["2"]["target"] = 0
trajectory_plan["0"]["correction"]["2"]["finished"] = False


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
    current_vel_t_left = ramp(current_vel_t_left, 0)
    current_vel_t_right = ramp(current_vel_t_right, 0)
    current_vel_angular = ramp(current_vel_angular, 0)


def process_correction(correction):
    global current_vel_x, current_vel_y, current_vel_z, current_vel_angular, current_vel_t_left, current_vel_t_right, refrences_dict
    if correction["type"] == "single":
        error = refrences_dict[correction["reference"]
                               ]["value"]-correction["target"]
        k = correction["controller_parameters"]["k"]
        bias = correction["controller_parameters"]["bias"]
        error_tolerance = correction["error_tolerance"]
        target_vel = (error) * k + (error)/abs(error) * bias

        if correction["reference"] == "401_x":
            # correct z
            if(abs(error) > error_tolerance):
                rospy.loginfo("correcting 401_x, error ="+str(error))
                current_vel_y = ramp(current_vel_y, target_vel)
                # current_vel_x = ramp(current_vel_x, 0)
                # current_vel_roll = ramp(current_vel_roll, 0)
                correction["finished"] = False
            else:
                current_vel_y = ramp(current_vel_y, 0)
                correction["finished"] = True
        elif correction["reference"] == "401_z":
            # correct x
            if(abs(error) > error_tolerance):
                rospy.loginfo("correcting 401_z, error ="+str(error))
                # current_vel_z = ramp(current_vel_z, 0)
                current_vel_x = ramp(current_vel_x, target_vel)
                # current_vel_roll = ramp(current_vel_roll, 0)
                correction["finished"] = False
            else:
                current_vel_x = ramp(current_vel_x, 0)
                correction["finished"] = True
        elif correction["reference"] == "401_pitch":
            # correct roll
            if(abs(error) > error_tolerance):
                # current_vel_z = ramp(current_vel_z, 0)
                # current_vel_x = ramp(current_vel_x, 0)
                rospy.loginfo("correcting 401_pitch, error ="+str(error))
                current_vel_angular = ramp(current_vel_angular, target_vel)
                correction["finished"] = False
            else:
                current_vel_angular = ramp(current_vel_angular, 0)
                correction["finished"] = True
        elif correction["reference"] == "401_yaw":
            # correct roll
            if(abs(error) > error_tolerance):
                # current_vel_z = ramp(current_vel_z, 0)
                # current_vel_x = ramp(current_vel_x, 0)
                rospy.loginfo("correcting 401_yaw, error ="+str(error))
                current_vel_t_left = ramp(current_vel_t_left, target_vel)
                current_vel_t_right = ramp(current_vel_t_right, target_vel)
                correction["finished"] = False
            else:
                current_vel_t_left = ramp(current_vel_t_left, 0)
                current_vel_t_right = ramp(current_vel_t_right, 0)
                correction["finished"] = True
        elif correction["reference"] == "401_y":
            # correct roll
            if(abs(error) > error_tolerance):
                # current_vel_z = ramp(current_vel_z, 0)
                # current_vel_x = ramp(current_vel_x, 0)
                rospy.loginfo("correcting 401_y, error ="+str(error))
                current_vel_t_left = ramp(current_vel_t_left, target_vel)
                current_vel_t_right = ramp(
                    current_vel_t_right, -1 * target_vel)
                correction["finished"] = False
            else:
                current_vel_t_left = ramp(current_vel_t_left, 0)
                current_vel_t_right = ramp(current_vel_t_right, 0)
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
        correction["finished"] = True
        for i in range(0, len(correction)-2):
            process_correction(correction[str(i)])
            if correction[str(i)]["finished"] == False:
                correction["finished"] = False
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
        difference_error = fl - fr

        if (fl < target and fr < target) or (fl > target and fr > target):
            # go forward or backward
            average_error = target - 0.5 * (fl+fr)
            target_vel = (average_error) * k_linear + \
                (float(average_error)/abs(average_error)) * bias_linear
            current_vel_x = ramp(current_vel_x, target_vel)
            current_vel_angular = ramp(current_vel_angular, 0)
        elif abs(difference_error) > correction["difference_pressure_tolerance"]:
            target_vel = (difference_error) * k_turn + \
                (float(difference_error)/abs(difference_error)) * bias_turn
            current_vel_x = ramp(current_vel_x, 0)
            current_vel_angular = ramp(current_vel_angular, target_vel)
        else:
            # stop
            current_vel_x = ramp(current_vel_x, 0)
            current_vel_angular = ramp(current_vel_angular, 0)

        # error_left = float(refrences_dict[correction["reference"]["left"]]["value"]-correction["target_left"])+0.000001
        # error_right = float(refrences_dict[correction["reference"]["right"]]["value"]-correction["target_right"])+0.000001
        # k = correction["controller_parameters"]["k"]
        # bias = correction["controller_parameters"]["bias"]
        # error_tolerance = correction["error_tolerance"]
        # target_vel_left = (error_left) * k + (float(error_left)/abs(error_left)) * bias
        # target_vel_right = (error_right) * k + (float(error_right)/abs(error_right)) * bias
        # target_vel_both = (target_vel_left + target_vel_right)
        # if (((abs(error_left) > error_tolerance) and (error_left > 0)) and (abs(error_right) < error_tolerance) ):
        #     #turn +
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular, -1 * abs(target_vel_left))
        # elif (((abs(error_right) > error_tolerance) and (error_right > 0)) and (abs(error_left) < error_tolerance) ):
        #     #turn -
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular,  abs(target_vel_right))
        # elif (((abs(error_left) > error_tolerance) and (error_left < 0)) and (abs(error_right) < error_tolerance) ):
        #     #turn -
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular,  abs(target_vel_left))
        # elif (((abs(error_right) > error_tolerance) and (error_right < 0)) and (abs(error_left) < error_tolerance) ):
        #     #turn +
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular,-1 *  abs(target_vel_right))
        # elif (((abs(error_left) > error_tolerance) and (error_left < 0)) and ((abs(error_right) > error_tolerance) and (error_right < 0)) ):
        #     # go forward
        #     current_vel_x = ramp(current_vel_x, abs(target_vel_both)*2)
        #     current_vel_angular = ramp(current_vel_angular, 0)
        # elif (((abs(error_left) > error_tolerance) and (error_left > 0)) and ((abs(error_right) > error_tolerance) and (error_right > 0)) ):
        #     # go backward
        #     current_vel_x = ramp(current_vel_x, -1 * abs(target_vel_both))
        #     current_vel_angular = ramp(current_vel_angular, 0)
        # elif (((abs(error_left) > error_tolerance) and (error_left > 0)) and ((abs(error_right) > error_tolerance) and (error_right < 0)) ):
        #     # turn +
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular,-1 *  (abs(target_vel_right)+abs(target_vel_left)))
        # elif (((abs(error_left) > error_tolerance) and (error_left < 0)) and ((abs(error_right) > error_tolerance) and (error_right > 0)) ):
        #     # turn -
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular, (abs(target_vel_right)+abs(target_vel_left)))

        # elif ((abs(error_left) < error_tolerance) and (abs(error_right) < error_tolerance) ):
        #     current_vel_x = ramp(current_vel_x, 0)
        #     current_vel_angular = ramp(current_vel_angular, 0)
        #     correction["finished"] = True

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
        if yaw < 0:
            yaw = math.pi * 2 - abs(yaw)
        refrences_dict[str(marker.id)+"_x"] = {"type": "marker", "value": x}
        refrences_dict[str(marker.id)+"_y"] = {"type": "marker", "value": y}
        refrences_dict[str(marker.id)+"_z"] = {"type": "marker", "value": z}
        refrences_dict[str(marker.id) +
                       "_roll"] = {"type": "marker", "value": roll}
        refrences_dict[str(marker.id) +
                       "_pitch"] = {"type": "marker", "value": pitch}
        refrences_dict[str(marker.id) +
                       "_yaw"] = {"type": "marker", "value": yaw}

        # print("yaw:"+str(yaw)+" pitch:"+str(pitch)," roll:"+str(roll))


def force_sensor_left_Callback(msg):
    global refrences_dict, filtering_value
    if "fsl" not in refrences_dict:
        refrences_dict["fsl"] = {"type": "force_sensor"}
        refrences_dict["fsl"]["value"] = msg.data
    else:
        refrences_dict["fsl"] = {"type": "force_sensor"}
        refrences_dict["fsl"]["value"] = msg.data
    # print(refrences_dict["fsl"]["value"])


def force_sensor_right_Callback(msg):
    global refrences_dict, filtering_value
    if "fsr" not in refrences_dict:
        refrences_dict["fsr"] = {"type": "force_sensor"}
        refrences_dict["fsr"]["value"] = msg.data
    else:
        refrences_dict["fsr"] = {"type": "force_sensor"}
        refrences_dict["fsr"]["value"] = msg.data
    # print(refrences_dict["fsr"]["value"])


def stop_signal_Callback(msg):
    global stop_signal
    stop_signal = msg.data


def sigint_handler(signum, frame):
    global cancel_signal
    cancel_signal = True


signal.signal(signal.SIGINT, sigint_handler)

rospy.init_node('approach_and_lift_controller', log_level=rospy.INFO)
cmd_vel_pub = rospy.Publisher('cmd_vel', Twist)
t_left_vel_pub = rospy.Publisher('traction_wheel_left_vel', Float64)
t_right_vel_pub = rospy.Publisher('traction_wheel_right_vel', Float64)
sub = rospy.Subscriber('aruco_marker_publisher/markers',
                       MarkerArray, MarkersCallback)
sub_markers_list = rospy.Subscriber(
    'aruco_marker_publisher/markers_list', UInt32MultiArray, Markers_list_Callback)
sub_force_sensor_left = rospy.Subscriber(
    'force_sensor_left', Int64, force_sensor_left_Callback)
sub_force_sensor_right = rospy.Subscriber(
    'force_sensor_right', Int64, force_sensor_right_Callback)
sub_stop_signal = rospy.Subscriber(
    'stop_signal', Bool, stop_signal_Callback)


def controller():
    global freq, refrences_dict, crnt_trgt_pnt_idx, trajectory_plan, current_vel_x, current_vel_y, current_vel_angular, refrences_dict_lock

    refrences_dict_lock = True
    if crnt_trgt_pnt_idx < len(trajectory_plan):
        current_point = trajectory_plan[str(crnt_trgt_pnt_idx)]
        if current_point["reached"] == True:  # point has already been reached
            crnt_trgt_pnt_idx = crnt_trgt_pnt_idx + 1  # move to next point
            rospy.loginfo("reached point"+str(crnt_trgt_pnt_idx))
        else:  # point has not been processed yet
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
                    if required_ref not in refrences_dict:
                        return False
                return True
            # rospy.loginfo("required_references:"+" ".join(required_reference_list))
            if check_all_references_available() == True:
                # rospy.loginfo("processing point: " + str(crnt_trgt_pnt_idx) + ", reference found")
                current_point["reached"] = process_correction(
                    current_point["correction"])
                print("left:", refrences_dict["fsl"]["value"],
                      " right:", refrences_dict["fsr"]["value"])
            else:  # reference marker not found, stop
                # rospy.loginfo("processing point: " + str(crnt_trgt_pnt_idx) + ", reference NOT found")
                stop()
    else:  # reached the end of the trajectory
        current_vel_x = ramp(current_vel_x, 0)
        current_vel_y = ramp(current_vel_y, 0)
        current_vel_angular = ramp(current_vel_angular, 0)

    refrences_dict_lock = False

    # rospy.loginfo(refrences_dict)
    # prepare actuating messages to be published
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

    # set the timer to fire up the same function in the specifc time period
    if cancel_signal != True:
        sleep_time = 1/float(freq)
        threading.Timer(sleep_time, controller).start()


controller()
r = rospy.Rate(freq)
while not rospy.is_shutdown():
    r.sleep()
