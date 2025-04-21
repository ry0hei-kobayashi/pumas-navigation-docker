#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Pose2D
import numpy as np

import sys
import roslib
sys.path.append(roslib.packages.get_pkg_dir("hma_env_manage") + "/script")
from lib_manage_env import libManageEnv
lme = libManageEnv()

#motion_synth for pumas
start_pose = {
    "arm_lift_joint": 0.0,
    "arm_flex_joint": np.deg2rad(0.0),
    "arm_roll_joint": np.deg2rad(0.0),
    "wrist_flex_joint": np.deg2rad(-90.0),
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": np.deg2rad(0.0),
}
goal_pose = {
    "arm_lift_joint": 0.4,
    "arm_flex_joint": np.deg2rad(-90.0),
    "arm_roll_joint": np.deg2rad(0.0),
    "wrist_flex_joint": np.deg2rad(-90.0),
    "wrist_roll_joint": 0.0,
    "head_pan_joint": 0.0,
    "head_tilt_joint": np.deg2rad(0.0),
}
ms_config = {"start_pose": start_pose, "goal_pose": goal_pose}

goal = Pose2D(0.3, 0.0, 0.0)
goal = Pose2D(2.0, 0.0, 0.0)
#lme.run(goal = goal, pose = ms_config, destination_name = None, first_rotate = False, 
#        nav_type = "pumas", nav_mode = "abs", nav_timeout = 0, goal_distance = 0,)
lme.run(goal=goal, pose=ms_config, destination_name=None, first_rotate=False, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0)


