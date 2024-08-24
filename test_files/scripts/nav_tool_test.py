#!/usr/bin/env python3

from navigation_tools.nav_tool_lib import NavModule
from geometry_msgs.msg import Pose2D

#call class at only first time
nav_module = NavModule("pumas")

pose = Pose2D(0.,0.,0.)
nav_module.nav_goal(pose)
#print(nav_module.is_moving())
