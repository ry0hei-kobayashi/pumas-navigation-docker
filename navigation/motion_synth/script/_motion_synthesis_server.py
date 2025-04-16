#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import copy
import os

import numpy as np
import rospy
from actionlib_msgs.msg import GoalStatusArray
from hsrlib.rosif import ROSInterfaces
from hsrlib.utils import joints as hsrjoints
from hsrlib.utils import locations
from motion_synth.msg import MotionSynthesisAction, MotionSynthesisResult
from tamlib.node_template import Node


class MotionSynthSrv(Node):
    def __init__(self) -> None:
        super().__init__()

        # Library
        self.rosif = ROSInterfaces()
        self.rosif.pub.auto_setup()
        self.rosif.action.auto_setup()

        # Subscriber
        self.status = GoalStatusArray()
        #TODO
        self.sub_register("status", "/move_base_flex/exe_path/status")

        # Action server
        self.action_server_register(
            "motion_synth", "/motion_synth", MotionSynthesisAction, self.run
        )

    def execute(self, location, pose, tolerance, is_changed_arm_flex=False) -> bool:
        while not rospy.is_shutdown():
            if self.action.server.motion_synthesis.is_preempt_requested():
                self.loginfo("Preempted")
                pose_dict = hsrjoints.hsr_joints_to_dict(pose)
                self.rosif.action.arm_controller(pose_dict, 0.1)
                self.rosif.action.head_controller(pose_dict, 0.1)
                hsrjoints.wait_for_all_joints(pose_dict, timeout=2.0)
                self.action.server.motion_synthesis.set_preempted()
                return False

            if is_changed_arm_flex is False:
                if locations.is_in_location(
                    (location.x, location.y, location.theta),
                    (tolerance.x, tolerance.y, 6.28),
                ):
                    _pose = copy.deepcopy(pose)
                    _pose.arm_flex_joint = (
                        0.0 if _pose.arm_flex_joint == 0.0 else -np.deg2rad(20)
                    )
                    pose_dict = hsrjoints.hsr_joints_to_dict(_pose)
                    self.rosif.pub.arm_command(pose_dict, 0.1)
                    self.rosif.pub.head_command(pose_dict, 0.1)
                    # res = hsrjoints.wait_for_all_joints(pose_dict, timeout=2.0)
                    # if res is False:
                    #     self.logwarn("wait_for_all_joints() TIMEOUT")
                    is_changed_arm_flex = True
            else:
                if locations.is_in_location(
                    (location.x, location.y, location.theta),
                    (tolerance.x, tolerance.y, np.deg2rad(45.0)),
                ):
                    pose_dict = hsrjoints.hsr_joints_to_dict(pose)
                    self.rosif.pub.arm_command(pose_dict, 0.1)
                    self.rosif.pub.head_command(pose_dict, 0.1)
                    res = hsrjoints.wait_for_all_joints(pose_dict, timeout=2.0)
                    if res is False:
                        self.logwarn("wait_for_all_joints() TIMEOUT")
                    return True

    def execute_from_first(self, pose, joints) -> None:
        go_pose = hsrjoints.get_go_pose()

        pose_dict = hsrjoints.hsr_joints_to_dict(pose)
        exec_pose = {}
        for joint in joints:
            exec_pose[joint] = pose_dict[joint]

        # LRFへの干渉を防ぐ
        if "arm_flex_joint" in exec_pose.keys():
            self.loginfo("LRF")
            min_arm_flex_joint = np.deg2rad(exec_pose["arm_lift_joint"] * -200 - 70)
            if exec_pose["arm_flex_joint"] < min_arm_flex_joint:
                exec_pose["arm_flex_joint"] = min_arm_flex_joint

        for key in go_pose.keys():
            if key not in exec_pose.keys():
                exec_pose[key] = go_pose[key]

        self.rosif.pub.arm_command(exec_pose, 0.1)
        self.rosif.pub.head_command(exec_pose, 0.1)

    def run(self, goal) -> None:
        if goal.exec_from_first == [] and goal.apply_start_pose:
            self.loginfo("Change to START pose")
            if not locations.is_in_location(
                (goal.goal_location.x, goal.goal_location.y, goal.goal_location.theta),
                (goal.goal_tolerance.x, goal.goal_tolerance.y, np.deg2rad(90.0)),
            ):
                self.rosif.pub.arm_command(
                    hsrjoints.hsr_joints_to_dict(goal.start_pose), 0.1
                )
                self.rosif.pub.head_command(
                    hsrjoints.hsr_joints_to_dict(goal.start_pose), 0.1
                )
            self.logsuccess("Change to START pose SUCCESS")

        res_exe = True
        if goal.apply_goal_pose:
            is_changed_arm_flex = False
            self.logwarn(goal.exec_from_first)
            if goal.exec_from_first != []:
                self.loginfo("Execute from first")
                self.execute_from_first(goal.goal_pose, goal.exec_from_first)
                if (
                    "arm_flex_joint" in goal.exec_from_first
                    and goal.goal_pose.arm_flex_joint < -0.1
                ):
                    is_changed_arm_flex = True
                    self.logsuccess("Execute from first SUCCESS")

            if locations.is_in_location(
                (goal.goal_location.x, goal.goal_location.y, goal.goal_location.theta),
                (0.3, 0.3, 1.57),
            ):
                is_changed_arm_flex = True

            self.loginfo("Change to GOAL pose")
            res_exe = self.execute(
                goal.goal_location,
                goal.goal_pose,
                goal.goal_tolerance,
                is_changed_arm_flex,
            )
            self.logsuccess("Change to GOAL pose SUCCESS")

        if res_exe:
            self.logsuccess("SUCCESS")
            result = MotionSynthesisResult(result=True)
            self.action.server.motion_synthesis.set_succeeded(result)
        else:
            self.loginfo("CANCELED")
            result = MotionSynthesisResult(result=False)
            self.action.server.motion_synthesis.set_succeeded(result)


def main():
    rospy.init_node(os.path.basename(__file__).split(".")[0])

    p_loop_rate = rospy.get_param(rospy.get_name() + "/loop_rate", 30)
    loop_wait = rospy.Rate(p_loop_rate)

    cls = MotionSynthSrv()
    rospy.on_shutdown(cls.delete)
    while not rospy.is_shutdown():
        try:
            pass
        except rospy.exceptions.ROSException:
            cls.logerr(f"[{rospy.get_name()}]: FAILURE")
        loop_wait.sleep()


if __name__ == "__main__":
    main()
