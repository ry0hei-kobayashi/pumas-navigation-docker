#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import sys
import rospy
import roslib
import smach
import math
from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import utils, description
from std_srvs.srv import Empty, EmptyRequest
from geometry_msgs.msg import Twist

# from hsrnavlib import LibHSRNavigation

import dynamic_reconfigure.client as reconf_client



class CreateMap(smach.State):
    """
    初期位置からマップを作成するための関数
    """
    def __init__(self):
        smach.State.__init__(self, outcomes=['success'], input_keys=[])
        self.hsrif = HSRInterfaces()
        self.description = description.load_robot_description()
    
    def rotate_robot(self):
    
        reconf_base = reconf_client.Client('tmc_map_merger/inputs/base_scan/obstacle_circle')
        reconf_head = reconf_client.Client('tmc_map_merger/inputs/head_rgbd_sensor/obstacle_circle')
        reconf_laser_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/head_rgbd_sensor')
        reconf_depth_obstacle_enable = reconf_client.Client('/tmc_map_merger/inputs/base_scan')

        reconf_laser_obstacle_enable.update_configuration({"enable": True})
        reconf_depth_obstacle_enable.update_configuration({"enable": False})

        rospy.sleep(1)

        # パブリッシャーの作成
        cmd_vel_pub = rospy.Publisher("/hsrb/command_velocity", Twist, queue_size=10)
        # メッセージのインスタンス化
        twist_msg = Twist()
        # 角速度の設定
        twist_msg.angular.z = math.pi / 12  # 45度/秒
        # 回転開始時間
        start_time = rospy.Time.now()
        # 一回転させるまでの時間
        rotate_duration = rospy.Duration.from_sec(math.pi * 2 / twist_msg.angular.z)  # 一周の時間 = 2π / 角速度
        # ループで角速度を送信し続ける
        while rospy.Time.now() - start_time < rotate_duration:
            cmd_vel_pub.publish(twist_msg)
            rospy.sleep(0.1)  # ループを遅延させる
        # ロボットを停止するために速度をゼロに設定
        twist_msg.angular.z = 0
        cmd_vel_pub.publish(twist_msg)

    def create_map(self):
        self.rotate_robot()

    def execute(self, userdata):
        self.create_map()
        return "success"
