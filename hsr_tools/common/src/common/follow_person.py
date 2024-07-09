#!/usr/bin/env python3
# -*- coding: utf-8 -*-
#

import rospy
import smach
import numpy as np

from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import utils, description, joints

from geometry_msgs.msg import Pose2D

from navigation_tools.nav_tool_lib import NavModule
#from common.speech import DefaultTTS

from std_msgs.msg import Bool
from geometry_msgs.msg import WrenchStamped

#hsrb 71,hsrc55
#THRESHOLD = 12.0
#hsrb 22
#THRESHOLD = 21.0


class FollowPerson(smach.State):
    def __init__(self, move_joints = False, timeout=None):
        smach.State.__init__(self, outcomes=['next', 'except'], input_keys=['way_point_list'], output_keys=['way_point_list'])

        self.hsrif = HSRInterfaces()

        self.nav = NavModule()
        
        #self.robot = robot
        #self.whole_body = self.robot.get('whole_body')
        #self.omni_base = self.robot.get("omni_base")
        #self.gripper = self.robot.get('gripper')

        #self._tts = DefaultTTS()


        self.fp_enable_leg_finder_pub = rospy.Publisher(
            '/hri/leg_finder/enable', Bool)
        self.fp_start_follow_pub = rospy.Publisher(
            '/hri/human_following/enable', Bool)

        self.fp_legs_found_sub = rospy.Subscriber(
            '/hri/leg_finder/legs_found', Bool, self._fp_legs_found_cb)


        self.fp_legs_found = False
        self.fisrt = True

        #false -> go pose , true -> custom_pose
        self.move_joints = move_joints

        #set custom_pose
        self.follow_pose = joints.get_go_pose()
        self.follow_pose["wrist_flex_joint"] = np.deg2rad(-90.0)
        self.follow_pose["wrist_roll_joint"] = np.deg2rad(0.0)
        self.follow_pose["arm_lift_joint"]   = 0.26
        self.follow_pose["arm_roll_joint"]   = 2.0
        self.follow_pose["arm_flex_joint"]   = -0.16


    def _fp_legs_found_cb(self, msg):
        try:
            if msg.data == True and msg.data != self.fp_legs_found:
                self.fp_legs_found = True

                self.hsrif.tts.say('I found you! Now, I will follow you.', language='en', sync=True, queue=True)
                
                if (self.fisrt):
                    self.hsrif.tts.say('If you want me to stop following you, push my hand.', language='en', sync=True, queue=True)
                    self.fisrt = False
                
                rospy.loginfo('Legs found')

            elif msg.data == False and msg.data != self.fp_legs_found:
                self.fp_legs_found = False

                self.hsrif.tts.say(
                    'Sorry, I lost you! Please come where I can see you.', language='en', sync=True, queue=True)
                rospy.loginfo('Legs lost')
        except:
            self.fp_legs_found = False

    def execute(self, userdata):
        try:
            rate = rospy.Rate(30)
            rospy.loginfo("exucute")
            
            if self.move_joints:
                self.hsrif.whole_body.move_to_joint_positions(self.follow_pose)
            else:
                self.hsrif.whole_body.move_to_go()

            rospy.loginfo("first pose")

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            self.hsrif.tts.say('Push my hand to start following you.', language='en', sync=True, queue=True)
            rospy.loginfo("Push my hand to start following you.")

            while not utils.is_arm_touched():
                rate.sleep()

            self.hsrif.tts.say('First I will find you. Please, move in front of me, where I can see you.', language='en', sync=True, queue=True)
            rospy.loginfo("First I will find you.")
            self.fp_enable_leg_finder_pub.publish(True)


            way_point_list = []

            last_pose_time = rospy.get_time()

            
            while not utils.is_arm_touched():
                current_time = rospy.get_time()
                #rospy.loginfo("current_time")
                rate.sleep()

                if current_time - last_pose_time >= 1:
                    nav_pose = self.nav.pose()
                    current_pose = Pose2D(nav_pose().x, nav_pose().y, nav_pose().theta)
                    

                    # current_pose_list = []                    
                    #current_pose = self.nav.pose() # List
                    #current_pose = [current_pose()]
                    # rospy.loginfo(current_pose)
                    # current_pose_list.append(current_pose)
                    
                    
                    if current_pose:
                        way_point_list.append(current_pose)
                        #rospy.loginfo(current_pose)
                        #rospy.loginfo(way_point_list)
                    last_pose_time = current_time

                if self.fp_legs_found == False:
                    self.fp_start_follow_pub.publish(False)

                    while self.fp_legs_found == False:

                        rate.sleep()

                    self.fp_start_follow_pub.publish(True)

                rate.sleep()

            userdata.way_point_list = way_point_list
            rospy.logwarn(userdata.way_point_list)

            self.fp_legs_found = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            self.hsrif.whole_body.move_to_go()

            self.hsrif.tts.say('OK, I will stop following you.', language='en', sync=True, queue=True)
            rospy.loginfo('OK, I will stop following you.')

            return 'next'

            # return 'timeout'
        except:
            import traceback
            traceback.print_exc()
            self.fp_legs_found = False

            self.fp_enable_leg_finder_pub.publish(False)
            self.fp_start_follow_pub.publish(False)

            return 'except'
