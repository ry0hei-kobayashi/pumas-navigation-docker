#!/usr/bin/env python3

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D

#call class at only first time
nav_module = NavModule("pumas")

# set to the goal point
goal = Pose2D(.0 ,.0 ,.0)
#goal = Pose2D(.0 , .0 , .0)

#call nav function 
while True:
    nav_module.nav_goal(goal, nav_type="pumas", nav_mode="abs", nav_timeout=0, goal_distance=0) # full definition

#nav_module.nav_goal(goal) # compact definition
