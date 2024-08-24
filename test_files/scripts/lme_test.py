#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D

#lib manage envをimport 
import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir("hma_env_manage") + "/script")
from lib_manage_env import libManageEnv
lme = libManageEnv()

#絶対移動, pumas navigation
goal = Pose2D(2.0, 0.0, 0.0)
lme.run(goal = goal, pose = None, destination_name = None, first_rotate = False, 
        nav_type = "pumas", nav_mode = "abs", nav_timeout = 0, goal_distance = 0)
