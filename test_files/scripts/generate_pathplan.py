#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import tf.transformations as tf_trans

from std_msgs.msg import Header
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Pose2D
from nav_msgs.msg import Path

class PathGenerator():

    def __init__(self):
        self.pub_path = rospy.Publisher("/simple_move/goal_path", Path, queue_size=10)

    def generate_new_path(self):

        # Pathメッセージの作成
        new_path = Path()
        new_path.header = Header()
        new_path.header.stamp = rospy.get_rostime()
        new_path.header.frame_id = "map"

        waypoints = [
            Pose2D(0.1, 0.1, 0.0),
            Pose2D(0.5, 0.25, 0.0),
            Pose2D(1.0, 0.5, 0.0),
            Pose2D(1.5, 0.75, 0.0),
            Pose2D(2.0, 1.1, 0.0),
            Pose2D(2.3, 0.6, 0.0)
        ]

        # 各ウェイポイントをPoseStampedに変換
        for waypoint in waypoints:
            pose_stamped = PoseStamped()
            pose_stamped.header = new_path.header
            pose_stamped.pose.position = Point(waypoint.x, waypoint.y, 0.0)

            # Pose2Dのthetaをeulerからquaternionに変換
            quaternion = tf_trans.quaternion_from_euler(0.0, 0.0, waypoint.theta)
            pose_stamped.pose.orientation = Quaternion(*quaternion)

            # new_pathに追加
            rospy.sleep(1) # important

            new_path.poses.append(pose_stamped)

        rospy.logwarn(new_path)

        # Pathのpublish
        self.pub_path.publish(new_path)
        rospy.loginfo("Path published")

if __name__ == '__main__':

    rospy.init_node('path_generator')

    path_generator = PathGenerator()
    path_generator.generate_new_path()

