#!/usr/bin/env python2.7
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose2D

# constraints of the motors velocities
maximum_abs_speed = 1000
maximum_distance_const = 40 # This is used to normalize the rotation velocity

#Information of the chosen reference frame (in cm)
lambda_ref = 60
x_ref = 0
z_ref = 0

#Here we need to indicate the Thetai,Zi,Xi,rotation_direction for every wheel.
theta = [0,-120,120]
z = [8.227, -4.114, -4.114] #in cm
x = [0, 7.125, -7.125] #in cm
rot_dir = [1, 1, 1]

#Equations parameters to determine the distances from center
a=[0,0,0]
b = -1
c=[0,0,0]
d=[0,0,0]

def calculate_matrices():
    global a,b,c,d,theta,z,x,rot_dir,lambda_ref,x_ref,z_ref

    # rospy.loginfo("x: " + str(x_ref)+" z: " + str(z_ref))
    for i in range(0,3):
        # global a,c,d
        # print(rad(theta[i]))
        a[i] = math.tan(math.radians(theta[i]))
        c[i] = z[i] - a[i] * x[i]
        d[i] = abs(a[i] * x_ref + b * z_ref + c[i]) / math.sqrt(a[i] * a[i] + b * b)


def omnidirectional_driver(vel_msg, pos_msg):
    global a,b,c,d,theta,z,x,rot_dir,lambda_ref,x_ref,z_ref
    # print("from the library",msg)
    final_vel=[0,0,0]
    W = vel_msg.angular.z 
    Vtx = vel_msg.linear.x
    Vtz = vel_msg.linear.y
    # lambda_ref = pos_msg.theta
    x_ref = pos_msg.x
    z_ref = pos_msg.y
    calculate_matrices()
    
    for i in range(0,3):
        # global final_vel
        Vroti = 0
        Vtransi = 0

        Vroti = (d[i] / maximum_distance_const) * W * rot_dir[i]
        # print(d[i])
        Vtransi = math.cos(math.radians(theta[i]) - math.radians(lambda_ref)) * Vtz + math.sin(math.radians(theta[i]) - math.radians(lambda_ref)) * Vtx
        Vi = Vtransi + Vroti
        final_vel[i]=Vi
    # print(final_vel)


    return final_vel[0],final_vel[1],final_vel[2]