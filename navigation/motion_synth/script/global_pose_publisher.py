#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os

import rospy
from geometry_msgs.msg import PoseStamped
from hsrlib.utils import description
from tamlib.node_template import Node
from tamlib.tf import Transform


class GlobalPosePublisher(Node):
    def __init__(self) -> None:
        super().__init__()

        # Library
        self.tamtf = Transform()
        self._description = description.load_robot_description()

        # Publisher
        self.pub_register("global_pose", "/global_pose", PoseStamped)

    def run(self) -> None:
        if self.run_enable is False:
            return

        global_pose = PoseStamped()
        global_pose.header.stamp = rospy.Time.now()
        global_pose.header.frame_id = self._description.frame.map
        global_pose.pose = self.tamtf.get_pose(
            self._description.frame.map, self._description.frame.base
        )
        self.pub.global_pose.publish(global_pose)


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", 100)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = GlobalPosePublisher()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            cls.run()
        except rospy.exceptions.ROSException:
            rospy.logerr(f"[{rospy.get_name()}]: FAILURE")
        loop_wait.sleep()


if __name__ == "__main__":
    main()
