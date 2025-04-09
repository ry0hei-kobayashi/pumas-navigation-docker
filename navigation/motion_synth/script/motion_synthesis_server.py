#!/usr/bin/env python3
import rospy
import copy
import numpy as np
import actionlib
from geometry_msgs.msg import Pose2D, Point, PoseStamped
from actionlib_msgs.msg import GoalStatus
from motion_synth.msg import MotionSynthesisAction, MotionSynthesisResult, MotionSynthesisFeedback
from std_msgs.msg import Float32MultiArray, Float32
from nav_msgs.msg import Path

from sensor_msgs.msg import JointState

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

        self.joint_states = {}
        self.joints_cb = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb)

        self.current_pose = None
        self.global_pose = rospy.Subscriber("/global_pose", PoseStamped, self.global_pose_callback)

    def wait_for_get_path(self):
        rospy.loginfo("motion_synth -> Waiting for get path")
        path_cb = None

        while not rospy.is_shutdown():
            try:
                msg = rospy.wait_for_message("/simple_move/goal_path", Path, timeout=1.0)
                if msg.poses and len(msg.poses) > 0:
                    first_pose = msg.poses[0].pose
                    if first_pose.position.x is not None and first_pose.position.y is not None:
                        path_cb = msg
                        break
            except rospy.ROSException:
                rospy.logwarn("motion_synth -> Timeout while wait_for_get_path... retrying")

            rospy.sleep(0.1)

        rospy.loginfo("motion_synth -> Got path")
        path_points = []
        for pose_stamped in path_cb.poses:
            x = pose_stamped.pose.position.x
            y = pose_stamped.pose.position.y
            path_points.append([x, y])

        rospy.logerr(len(path_points))
        self.path_len = len(path_points)
        self.path_points = path_points

        return path_points

    def global_pose_callback(self, msg):
        self.current_pose = [msg.pose.position.x, msg.pose.position.y]

    def joint_state_cb(self, msg):
        for name, pos in zip(msg.name, msg.position):
            self.joint_states[name] = pos

    def joint_goal_reached(self, goal_joints, threshold=0.15):
        def get(name): return self.joint_states.get(name, float("inf"))

        diffs = [
            #abs(goal_joints.arm_lift_joint - get("arm_lift_joint")),
            abs(goal_joints.arm_flex_joint - get("arm_flex_joint")),
            abs(goal_joints.arm_roll_joint - get("arm_roll_joint")),
            abs(goal_joints.wrist_flex_joint - get("wrist_flex_joint")),
            abs(goal_joints.wrist_roll_joint - get("wrist_roll_joint")),
            abs(goal_joints.head_pan_joint - get("head_pan_joint")),
            abs(goal_joints.head_tilt_joint - get("head_tilt_joint")),
        ]
        return all(d < threshold for d in diffs)

    def execute_cb(self, goal):
        path = self.wait_for_get_path()
        arm_start_position = int(len(self.path_points) * 0.75)
        print("start_timing", arm_start_position)
        triggered =False
        
        rospy.loginfo(f"motion_synth -> Goal received (x={goal.goal_location.x}, y={goal.goal_location.y})")

        if goal.apply_start_pose:
            self.move_to_pose(goal.start_pose, "Start Pose")
            rospy.sleep(0.1)

        if goal.apply_goal_pose:
            self.move_to_pose(goal.goal_pose, "End Pose")
            rospy.sleep(0.1)

        feedback = MotionSynthesisFeedback()
        rate = rospy.Rate(10)
        timeout = rospy.Time.now() + rospy.Duration(10.0) #TODO

        while not rospy.is_shutdown():
            #print(goal.goal_pose)

            # 3/4地点到達判定
            if not triggered and self.current_pose is not None:
                cx, cy = self.current_pose
                tx, ty = self.path_points[arm_start_position]
                dist = np.linalg.norm([cx - tx, cy - ty])
                if dist < 0.15:  # 閾値調整可能
                    rospy.loginfo("motion_synth -> Reached 3/4 of path. Triggering end pose")
                    self.move_to_pose(goal.goal_pose, "End Pose (3/4 Trigger)")
                    triggered = True

            if self.server.is_preempt_requested():
                rospy.logwarn("motion_synth -> Preempt requested")
                self.server.set_preempted()
                return

            if self.joint_goal_reached(goal.goal_pose):
                rospy.loginfo("motion_synth -> Arm Goal Reached")
                result = MotionSynthesisResult(result=True)
                self.server.set_succeeded(result)

            if rospy.Time.now() > timeout:
                rospy.logwarn("motion_synth -> Timeout")
                self.server.set_aborted(MotionSynthesisResult(result=False), "Timeout")
                return

            #feedback.feedback = 1.0  # Example
            self.server.publish_feedback(feedback)
            rate.sleep()


    def move_to_pose(self, joints, description):

        #print("##########################")
        #print(joints)
        #print("##########################")

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

        #rospy.sleep(2.0)


if __name__ == "__main__":
    MotionSynthesisServer()
    rospy.spin()

