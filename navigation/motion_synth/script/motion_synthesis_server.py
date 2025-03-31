#!/usr/bin/env python3
import rospy
import copy
import numpy as np
import actionlib
from geometry_msgs.msg import Pose2D, Point
from actionlib_msgs.msg import GoalStatus
from motion_synth.msg import MotionSynthesisAction, MotionSynthesisResult, MotionSynthesisFeedback
from std_msgs.msg import Float32MultiArray, Float32

class MotionSynthesisServer:
    def __init__(self):
        rospy.init_node("motion_synth_server")

        self.server = actionlib.SimpleActionServer(
            "/motion_synth",
            MotionSynthesisAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self.server.start()
        rospy.loginfo("motion_synth -> Action Server Ready")

        self.arm_pub = rospy.Publisher("/hardware/arm/goal_pose", Float32MultiArray, queue_size=1)
        self.lift_pub = rospy.Publisher("/hardware/torso/goal_pose", Float32, queue_size=1)
        self.head_pub = rospy.Publisher("/hardware/head/goal_pose", Float32MultiArray, queue_size=1)

    def execute_cb(self, goal):
        rospy.loginfo(f"motion_synth -> Goal received (x={goal.goal_location.x}, y={goal.goal_location.y})")

        if goal.apply_start_pose:
            self.move_to_pose(goal.start_pose, "Start Pose")

        feedback = MotionSynthesisFeedback()
        is_goal_sent = False

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.server.is_preempt_requested():
                rospy.logwarn("motion_synth -> Preempt requested")
                self.server.set_preempted()
                return
            print(goal.goal_pose)

            if goal.apply_goal_pose and not is_goal_sent:
                #TODO
                self.move_to_pose(goal.goal_pose, "Goal Pose")
                is_goal_sent = True

            feedback.feedback = 1.0  # Example
            self.server.publish_feedback(feedback)
            rate.sleep()

        result = MotionSynthesisResult(result=True)
        self.server.set_succeeded(result)

    def move_to_pose(self, joints, description):

        print("##########################")
        print(joints)
        print("##########################")

        rospy.loginfo(f"motion_synth -> Moving to {description}")

        lift_msg = Float32()
        lift_msg.data = joints.arm_lift_joint

        arm_msg = Float32MultiArray()
        arm_msg.data = [
            joints.arm_flex_joint,
            joints.arm_roll_joint,
            joints.wrist_flex_joint,
            joints.wrist_roll_joint,
        ]

        head_msg = Float32MultiArray()
        head_msg.data = [
            joints.head_pan_joint,
            joints.head_tilt_joint,

        ]

        self.lift_pub.publish(lift_msg)
        self.arm_pub.publish(arm_msg)
        self.head_pub.publish(head_msg)

        rospy.sleep(2.0)


if __name__ == "__main__":
    MotionSynthesisServer()
    rospy.spin()

