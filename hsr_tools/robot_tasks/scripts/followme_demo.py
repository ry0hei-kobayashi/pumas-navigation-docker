#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS library
import rospy
import smach
import traceback
import logging

# HSRB Library
#import hsrb_interface
from hsrlib.hsrif import HSRInterfaces
from hsrlib.utils import utils

from tf.transformations import quaternion_matrix, quaternion_from_euler, quaternion_from_matrix

#from common import speech
from common.follow_person import FollowPerson
#from common.wait_hand_pushed import WaitHandPushed

from navigation_tools.nav_tool_lib import nav_module

##################################################

## Main
#robot = hsrb_interface.Robot()
#
#whole_body = robot.get("whole_body")
##omni_base = robot.get("omni_base") #Standard initialisation (Toyota)
#omni_base = nav_module("hsr") #New initalisation (Pumas)
#gripper = robot.get('gripper')
#tf_buffer = robot._get_tf2_buffer()
#
#default_tts = speech.DefaultTTS()
#console = speech.Console()
#SAY = default_tts.say
#
#whole_body.move_to_neutral()

hsrif = HSRInterfaces()

omni_base = nav_module("hsr") #New initalisation (Pumas)
hsrif.whole_body.move_to_neutral()

rospy.loginfo('initializing...')
rospy.sleep(3)

def create_sm():

  sm = smach.StateMachine(outcomes=['next', 'except'])

  with sm:

        ##########
        # START: TASK INITIALISATION
        ##########
        # Ask for the hand to be pushed to start


        @smach.cb_interface(outcomes=['next', 'except'])
        def wait4start_cb(userdata):
            try:
                hsrif.tts.say('手が押されたら働くのだ', language='ja', sync=True, queue=True)
                while not rospy.is_shutdown():
                    if utils.is_arm_touched():
                        break
                        
                rospy.sleep(0.1)
                hsrif.whole_body.move_to_neutral()
                hsrif.tts.say('準備しているのだ', language='ja', sync=True, queue=True)
                rospy.sleep(1)
                return 'next'
            except:
                return 'except'

        smach.StateMachine.add('Wait4Start', smach.CBState(wait4start_cb),
                               transitions={'next': 'FOLLOWPERSON',
                                            'except': 'except'})
        ##########
        # END: TASK INITIALISATION
        ##########

        ##########
        #START: FOLLOW PERSON
        ##########
        smach.StateMachine.add('FOLLOWPERSON', FollowPerson(move_go=True),
                               transitions = {'next': 'MOVE2STANDBY',
                                              'except': 'except'})
        ##########
        #END: FOLLOW PERSON
        ##########

        ##########
        # START: BASIC MOVE SEQUENCE
        ##########
        # Moves the robot to the neutral position
        @smach.cb_interface(outcomes=['next', 'except'])
        def move_to_standby_cb(userdata):
            try:
                hsrif.whole_body.move_to_neutral()

                return 'next'
            except:
                return 'except'

        smach.StateMachine.add('MOVE2STANDBY', smach.CBState(move_to_standby_cb),
                               transitions={'next': 'next',
                                            'except': 'except'})

        # Moves the robot to the neutral position
        @smach.cb_interface(outcomes=['next', 'except'])
        def move_to_timeout_cb(userdata):
            try:
                hsrif.whole_body.move_to_neutral()
                hsrif.tts.say('タイムアウトしたのだ', language='ja', sync=True, queue=True)
                rospy.sleep(2)

                return 'next'
            except:
                return 'except'

        smach.StateMachine.add('MOVE2TIMEOUT', smach.CBState(move_to_timeout_cb),
                               transitions={'next': 'next',
                                            'except': 'except'})
        ##########
        # END: BASIC MOVE SEQUENCE
        ##########

  return sm


if __name__ == '__main__':
    
    sm = create_sm()
    outcome = sm.execute()
    
    if outcome == 'next':
        hsrif.tts.say('タスク完了なのだ', language='ja', sync=True, queue=True)

        rospy.loginfo('I finished the task.')
    else:
        hsrif.tts.say('無理なのだ諦めたのだ', language='ja', sync=True, queue=True)
        rospy.signal_shutdown('Some error occured.')
    
    logging.disable(logging.CRITICAL)
