#! /usr/bin/env python3

import tf
import tf2_ros
import numpy as np
import rospy
import actionlib
from hmm_navigation.msg import NavigateAction ,NavigateActionGoal,NavigateActionFeedback,NavigateActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker , MarkerArray

##################################################
def pose2feedback(pose_robot,quat_robot):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot   = pose_robot[0]
    feed.feedback.y_robot   = pose_robot[1]
    euler= tf.transformations.euler_from_quaternion((quat_robot[0] ,quat_robot[1] ,quat_robot[2] ,quat_robot[3] )) 
    feed.feedback.yaw_robot = euler[2]
    feed.feedback.status    = 3
    return feed
class pumas_navServer():

    def __init__(self):
        self.pumas_nav_server = actionlib.SimpleActionServer("navigate",NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self.pumas_nav_server.start()
    
  
    def execute_cb(self, goal):
        
        print (goal)
        x,y,yaw=goal.x ,goal.y,goal.yaw

        success = True
        result = NavigateActionResult()
        rate = rospy.Rate(1)
        timeout= rospy.Time.now().to_sec()+goal.timeout
        
        rot=tf.transformations.quaternion_from_euler(0,0,yaw)
        goal_pose= PoseStamped()
        goal_pose.pose.position.x=x
        goal_pose.pose.position.y=y
        goal_pose.pose.orientation.z=rot[0]
        goal_pose.pose.orientation.z=rot[1]
        goal_pose.pose.orientation.z=rot[2]
        goal_pose.pose.orientation.w=rot[3]
        goal_nav_publish.publish(goal_pose)

        #goal_pnt= PointStamped()
        #goal_pnt.header.frame_id='map'
        #goal_pnt.point.x , goal_pnt.point.y  =x,y
        #pub_goal.publish(goal_pnt)

        
        i=0
        while timeout >= rospy.Time.now().to_sec():     
            i+=1
            
            
            try:
                pose_robot,quat_robot=listener.lookupTransform('map', 'base_footprint', rospy.Time(0)) 
            except:
                print ('notf')
                pose_robot=np.zeros(3)
                quat_robot= np.zeros(4)
                quat_robot[-1]=1

            feed = pose2feedback(pose_robot,quat_robot)
            self.hmm_nav_server.publish_feedback(feed.feedback)
        
            euclD=   np.linalg.norm(np.asarray((x,y))- pose_robot[:2])
            if euclD<=0.2:
                print ('Close Enough')  
                result.result.success=1
                self.pumas_nav_server.set_succeeded(result.result)  
                break
            if i ==10000:
                print (euclD)
                i=0
            
        goal_pnt.point.x , goal_pnt.point.y  =0,0
        pub_goal.publish(goal_pnt)

            
        
        if result.result.success!=1:
            print('time is over')
            result.result.success=2
            #print (result)
        self.pumas_nav_server.set_succeeded(result.result)

            

        












        
if __name__=="__main__":
    global listener , goal_nav_publish_pumas
    rospy.init_node('pumas_navigation_actionlib_server')
    
    goal_nav_publish_pumas = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
    #pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    #pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
    #pub3= rospy.Publisher('aa/Markov_route',MarkerArray,queue_size=1)
    #pub_goal= rospy.Publisher('/clicked_point',PointStamped,queue_size=1)
    listener = tf.TransformListener()
    
    
    s = pumas_navServer()
    rospy.spin()

    