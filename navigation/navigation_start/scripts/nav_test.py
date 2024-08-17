#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D

#lib manage envをimport 
import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir("hma_env_manage") + "/script")
from lib_manage_env import libManageEnv
lme = libManageEnv()

#dynamic_reconfigureの定義(hsr標準navigation,omni_base用)
import dynamic_reconfigure.client as reconf_client
reconf_base = reconf_client.Client('tmc_map_merger/inputs/base_scan/obstacle_circle')
reconf_head = reconf_client.Client('tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle')
reconf_depth_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/head_rgbd_sensor')
reconf_laser_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/base_scan')

#Falseの時dynamic_obstacle_mapの更新を無視，ドアなど狭いところをrelで通りたいときなど
reconf_laser_obstacle_enable.update_configuration({"enable":False})
reconf_depth_obstacle_enable.update_configuration({"enable":False})

#現在の座標からx座標方向に2m相対移動,hsr navigation rel mode
goal = Pose2D(2.0, 0.0, 0.0)
lme.run(goal = goal, pose = None, destination_name = None, first_rotate = False, 
        nav_type = "hsr", nav_mode = "rel", nav_timeout = 0, goal_distance = 0)

#Trueの時dynamic_obstacle_mapの更新,障害物更新
reconf_laser_obstacle_enable.update_configuration({"enable":True})
reconf_depth_obstacle_enable.update_configuration({"enable":True})

#dynamic_obstacle_mapのパラメータを変更する,guiで変更する場合rosrun rqt_reconfigure rqt_reconfigure
reconf_base.update_configuration({"forbid_radius":0.15, "obstacle_radius":0.20, "obstacle_occupancy":5})

#絶対移動, hsr navigation
goal = Pose2D(5.0, 5.0, 0.0)
lme.run(goal = goal, pose = None, destination_name = None, first_rotate = False, 
        nav_type = "hsr", nav_mode = "abs", nav_timeout = 0, goal_distance = 0)

#絶対移動, pumas navigation
goal = Pose2D(2.0, 0.0, 0.0)
lme.run(goal = goal, pose = None, destination_name = None, first_rotate = False, 
        nav_type = "pumas", nav_mode = "abs", nav_timeout = 0, goal_distance = 0)
