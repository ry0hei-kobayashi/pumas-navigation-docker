#!/usr/bin/env python3

import rospy
import dynamic_reconfigure.client as reconf_client

import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir("hma_env_manage") + "/script")
from lib_manage_env import libManageEnv
from geometry_msgs.msg import Pose2D
lme = libManageEnv()

reconf_base = reconf_client.Client('tmc_map_merger/inputs/base_scan/obstacle_circle')
reconf_head = reconf_client.Client('tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle')
reconf_depth_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/head_rgbd_sensor')
reconf_laser_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/base_scan')

##disable/enable xtion and urg obstacle avoidance
reconf_laser_obstacle_enable.update_configuration({"enable":True})
reconf_depth_obstacle_enable.update_configuration({"enable":True})


goal = Pose2D(2.0, 0.0, 0.0)
lme.run(goal = goal, pose = None, destination_name = None, first_rotate = False, nav_type = "hsr", nav_mode = "rel", nav_timeout = 0, goal_distance = 0)

