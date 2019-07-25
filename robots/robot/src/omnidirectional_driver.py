#!/usr/bin/env python2.7
import rospy
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist



# constraints of the motors velocities
maximum_abs_speed = 1000
maximum_distance_const = 40 # This is used to normalize the rotation velocity

#Information of the chosen reference frame (in cm)
lambda_ref = 60
x_ref = 0
z_ref = 0

#Here we need to indicate the Thetai,Zi,Xi,rotation_direction for every wheel.
theta = [0,-120,120]
z = [7, -3.5, -3.5] #in cm
x = [0, 6, -6] #in cm
rot_dir = [1, 1, 1]

#Equations parameters to determine the distances from center
a=[0,0,0]
b = -1
c=[0,0,0]
d=[0,0,0]

#Inputs for Omnidirectional drivers from transmitor
Vtz_manual = 0
Vtx_manual = 0
W_manual = 0

#Inputs for Omnidirectional drivers automated
Vtz_automated = 0
Vtx_automated = 0
W_automated = 0




# def rad(deg):
#   rad = deg * 1000 / 57296
#   return rad

for i in range(0,3):
    # global a,c,d
    # print(rad(theta[i]))
    a[i] = math.tan(math.radians(theta[i]))
    c[i] = z[i] - a[i] * x[i]
    d[i] = abs(a[i] * x_ref + b * z_ref + c[i]) / math.sqrt(a[i] * a[i] + b * b)


def omnidirectional_driver(msg):
    # global a,b,c,d,theta,z,x,rot_dir,lambda_ref,x_ref,z_ref
    # print("from the library",msg)
    final_vel=[0,0,0]
    W = msg.angular.z 
    Vtx = msg.linear.x
    Vtz = msg.linear.y
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