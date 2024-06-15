#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019
Updated on Mon May 06 15:02:03 2024

@author: oscar
"""

import rospy
import numpy as np
from geometry_msgs.msg import Twist, PointStamped
from sensor_msgs.msg import LaserScan
import tf2_ros
from tf.transformations import euler_from_quaternion


LIN_ACC = 0.0015
LIN_DES = - 0.0005

MAX_LIN_SPEED_HIGH = 0.45
MAX_LIN_SPEED_MID = 0.08

ANG_ACC_LOW = 0.001
ANG_ACC_MID = 0.003
ANG_ACC_HIGH = 0.006

MAX_ANG_SPEED_LOW = 0.2
MAX_ANG_SPEED_MID = 0.4
MAX_ANG_SPEED_HIGH = 0.6

FORCE_THRESHOLD_LOW = np.pi/10
FORCE_THRESHOLD_MID = np.pi/2
FORCE_THRESHOLD_HIGH = np.pi * (4/5)
xcl, ycl = 0, 0
Fx_rep, Fy_rep = 0, 0

# Transform tf message to np arrays
def tf_2_np_array(t):
    pose = np.asarray((
        t.transform.translation.x,
        t.transform.translation.y,
        t.transform.translation.z))
    quat = np.asarray((
        t.transform.rotation.x,
        t.transform.rotation.y,
        t.transform.rotation.z,
        t.transform.rotation.w))
    return pose, quat

def get_robot_current_pose():
        # Get current robot position
    try:
        trans = tfBuffer.lookup_transform('map','base_footprint', rospy.Time(), rospy.Duration(5.0))
        pose, quat = tf_2_np_array(trans)
        x, y = pose[0], pose[1]
        euler = euler_from_quaternion(quat)
        th = euler[2]
    
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
        rospy.WARN('Waiting for TF')
        x, y = 0,0
        th = 0
    return x, y, th

def calculate_force(enable_repulsion = True):
    
    # No entiendo por que hay que sumar una cantidad muy pequeña !!!
    if enable_repulsion:
        Fth_rep = np.arctan2(Fy_rep, Fx_rep) + np.pi
        Fmag_rep = np.linalg.norm((Fx_rep, Fy_rep))
    else:
        Fth_rep = 0
        Fmag_rep = 0
    x, y, th = get_robot_current_pose()

    # Calculate atractive force
    xy, xycl = np.array((x,y)), np.array((xcl,ycl))
    euclD = np.linalg.norm(xy - xycl)

    Fx_atr = -(x - xcl) / euclD
    Fy_atr = -(y - ycl) / euclD      
    Fth_atr = np.arctan2(Fy_atr, Fx_atr) 
    Fth_atr = Fth_atr - th
    Fmag_atr = np.linalg.norm((Fx_atr, Fy_atr))

    Fx_tot = Fmag_rep * np.cos(Fth_rep) * 0.0025 + Fmag_atr * np.cos(Fth_atr)
    Fy_tot = Fmag_rep * np.sin(Fth_rep) * 0.0025 + Fmag_atr * np.sin(Fth_atr)
    Fth_tot = np.arctan2(Fy_tot, Fx_tot)

    return Fx_tot, Fy_tot, Fth_tot, euclD

def speed_behavior(current_speed, Fx, Fy, Fth, distance):
    speed = Twist()
    
    # Speed behaviors
    if abs(Fth) < FORCE_THRESHOLD_LOW:
        speed.linear.x =  min(current_speed.linear.x + LIN_ACC, MAX_LIN_SPEED_HIGH)
        speed.angular.z = 0
        print('linear movement')

    elif abs(Fth) < FORCE_THRESHOLD_MID and abs(Fth) > FORCE_THRESHOLD_LOW:
        print('Angular movement (low)')
        speed.linear.x  = max(current_speed.linear.x + LIN_DES, MAX_LIN_SPEED_MID)
        speed.angular.z = max(current_speed.angular.z + ANG_ACC_LOW * np.sign(Fth), 
                                MAX_ANG_SPEED_LOW * np.sign(Fth))

    elif abs(Fth) < FORCE_THRESHOLD_HIGH and abs(Fth) > FORCE_THRESHOLD_MID:
        print('Angular movement (mid)')
        speed.linear.x  = 0#max(current_speed.linear.x + LIN_DES * 3.5, MAX_LIN_SPEED_MID)
        speed.angular.z = max(current_speed.angular.z + ANG_ACC_MID * np.sign(Fth), 
                                MAX_ANG_SPEED_MID * np.sign(Fth))

    elif abs(Fth) > FORCE_THRESHOLD_HIGH:
        print('Angular movement (high)')
        speed.linear.x  = 0 #current_speed.linear.x + LIN_DES * 5 #max(current_speed.linear.x + LIN_DES * 4, MAX_LIN_SPEED_MID)
        speed.angular.z = max(current_speed.angular.z + ANG_ACC_HIGH * np.sign(Fth), 
                                MAX_ANG_SPEED_HIGH * np.sign(Fth))
    return speed

def final_turn(current_speed, Fth):
    speed = Twist()
    speed.angular.z = max(current_speed.angular.z + ANG_ACC_MID * np.sign(Fth), 
                                MAX_ANG_SPEED_LOW * np.sign(Fth))
    return speed

def calculate_weight(num_readings):
    center = num_readings / 2
    weights = np.array([-(x - center)**2 + center**2 for x in range(num_readings)])
    return weights / np.max(weights)

# Suscribers callbacks

def read_point_cb(msg):
    global xcl,ycl
    xcl = msg.point.x
    ycl = msg.point.y
           
def read_sensor_cb(msg):
    # This callback is about to be shorten
    global xcl,ycl, Fx_rep, Fy_rep
    lectures = np.asarray(msg.ranges)
    lectures[np.isinf(lectures)] = 13.5

    # No entiendo por que Fy_rep no inicia en 0 como los otros !!!
    Fx_rep = 0.0
    Fy_rep = 0.0

    # Calculate repulsive force
    for idx, deg in enumerate(laser_degs):            
        Fx_rep = Fx_rep + (1/lectures[idx])**2 * np.cos(deg)
        Fy_rep = Fy_rep + (1/lectures[idx])**2 * np.sin(deg)

# Cambio de nombre de inoutinout a main #:D:D:D
def main():
    global listener, tfBuffer, xcl, ycl, laser_degs

    rospy.init_node('pot_fields_nav', anonymous = True)

    # Setup variables
    rate = rospy.Rate(25) # 10hz
    data = rospy.wait_for_message('/hsrb/base_scan', LaserScan, timeout=5)
    laser_degs = np.linspace(data.angle_min, data.angle_max, len(data.ranges))

    # Subscribers and publishers
    laser_base_sub = rospy.Subscriber("/hsrb/base_scan", LaserScan, read_sensor_cb)
    clicked_point_sub = rospy.Subscriber("/clicked_point", PointStamped, read_point_cb)
    cmd_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    
    # TF buffer and listener
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    
    rospy.loginfo('Pot Fields Navigation AMCL active')

    current_speed = Twist()
    enable_repulsion = True

    while not rospy.is_shutdown():
        if (xcl != 0 and ycl != 0):
            
            # Debug prints
            # print("xrob, yrob, throbot: {:.2f}, {:.2f}, {:.2f}".format(x, y, np.rad2deg(th)))
            # print("xclick, yclick: {:.2f}, {:.2f}, euclD: {:.2f}".format(xcl, ycl, euclD))
            # print("Repulsive Force: Fx, Fy, Fth: {:.2f}, {:.2f}, {:.2f}".format(Fx_rep, Fy_rep, np.rad2deg(Fth_rep)))
            # print("Atractive Force: Fx, Fy, Fth: {:.2f}, {:.2f}, {:.2f}".format(Fx_atr, Fy_atr,  np.rad2deg(Fth_atr)))
            # print("Total force Fx, Fy, Tth: {:.2f}, {:.2f}".format(Fx_tot, Fy_tot, np.rad2deg(Fth_tot)))

            Fx, Fy, Fth, euclD = calculate_force(enable_repulsion)
            
            #El threshold es alto porque el goal siempre es una persona y se busca no atropellarla
            proximity_threshold = 1.0 # 0.15 
            
            # Stop condition
            if(euclD < proximity_threshold):
                current_speed.linear.x = 0
                current_speed.linear.y = 0
                #current_speed.angular.z = 0
                enable_repulsion = False
                print("Arrived, missing final turn!")
                if abs(Fth) > 0.1:
                    current_speed = final_turn(current_speed, Fth)
                else:
                    xcl, ycl = 0.0, 0.0
                    print("Navigation successful!")
            
            else:
                current_speed = speed_behavior(current_speed, Fx, Fy, Fth, euclD)
                enable_repulsion = True

            
            cmd_vel_pub.publish(current_speed)
            rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
