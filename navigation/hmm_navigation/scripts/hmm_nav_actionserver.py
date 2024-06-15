#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Dec 17 02:14:10 2022

@author: oscar
"""
from hmm_nav_utils import *

class HMM_navServer():

    def __init__(self):
        self.hmm_nav_server = actionlib.SimpleActionServer("navigate_hmm",NavigateAction, execute_cb=self.execute_cb, auto_start=False)
        self.hmm_nav_server.start()
    
  
    def execute_cb(self, goal):
        
        print (goal)
        x,y,th=goal.x ,goal.y,goal.yaw
        pose_robot,quat_robot=listener.lookupTransform('map', 'base_footprint', rospy.Time(0)) 
        th_robot= tf.transformations.euler_from_quaternion(quat_robot)[2]
        xyth= np.asarray((pose_robot[0],pose_robot[1],th_robot)) 

        euclD=   np.linalg.norm(np.asarray((x,y))- pose_robot[:2])
        print ('xyth,euclD',xyth,euclD)
        result = NavigateActionResult()




        success = True
        
        #rate = rospy.Rate(1)
        timeout= rospy.Time.now().to_sec()+goal.timeout
        goal_pnt= PointStamped()
        goal_pnt.header.stamp=rospy.Time.now()
        goal_pnt.header.frame_id='map'
        goal_pnt.point.x , goal_pnt.point.y  =x,y
        pub_goal.publish(goal_pnt)
        print (goal_pnt)
        _,xythcuant= quantized(xyth,ccxyth)
        _,xythclcuant= quantized(np.asarray((x,y,th)),ccxyth)
        
        path=[]
        if (xythcuant!=xythclcuant):path=dijkstra(xythcuant,xythclcuant,Markov_A_2_grafo(A,ccxyth))
        
        print ('path',path) 


                    
                #result.result.success=1
                #self.hmm_nav_server.set_succeeded(result.result)
                
        result.result.success=2
        i=0



        while timeout >= rospy.Time.now().to_sec():     
            
            
            
            i+=1
            
            pose_robot,quat_robot=listener.lookupTransform('map', 'base_footprint', rospy.Time(0)) 
            
            th_robot= tf.transformations.euler_from_quaternion(quat_robot)[2]
            xyth= np.asarray((pose_robot[0],pose_robot[1],th_robot)) 
            _,xythcuant= quantized(xyth,ccxyth)
            

            euclD=   np.linalg.norm(np.asarray((goal.x ,goal.y))- pose_robot[:2])

            
        
            if euclD<=0.4:
                print ('Close Enough')  
                print ('xyth,euclD',xyth,euclD,np.asarray((x,y)))

                result.result.success=1
                break

            ####################
            
            if len (path)!=0:x_nxt,y_nxt,th_nxt= ccxyth[path[0]]
            else:x_nxt,y_nxt,th_nxt = x,y,0.0
            xyth_nxt=np.array((x_nxt,y_nxt,th_nxt))
            _,xyth_nxt_cuant= quantized(xyth_nxt,ccxyth)

            

            if (xythcuant in path[1:]):
                killix= path.index(xythcuant)
                print ('SHortuct DETECTED',killix)
                del path[:path.index(xythcuant)]


            if ((  xythcuant==xyth_nxt_cuant  or np.linalg.norm(ccxyth[xythcuant][:2]- ccxyth[xyth_nxt_cuant][:2])<.4  )  ):

                #print ('check node. Activate next')
                if (len (path)==1):
                    #print('PATH LEN 0 WTF ******* ')
                    x,y,th=goal.x ,goal.y,goal.yaw
                else:
                    path.pop(0)
                    x,y,th= ccxyth[path[0]]
                goal_pnt= PointStamped()
                goal_pnt.header.stamp=rospy.Time.now()
                goal_pnt.header.frame_id='map'
                goal_pnt.point.x , goal_pnt.point.y  =x,y
                pub_goal.publish(goal_pnt)


            
            

                

            
            if i ==1000:
                timeleft=timeout-rospy.Time.now().to_sec()     
            
                feed = pose2feedback(pose_robot,quat_robot,timeleft,euclD)
                self.hmm_nav_server.publish_feedback(feed.feedback)
                
                print (timeleft,euclD, 'xythcuant','xythclcuant',xythcuant,xythclcuant,path)
                i=0
            
        goal_pnt.point.x , goal_pnt.point.y  =0,0
        pub_goal.publish(goal_pnt)
        if result.result.success!=1:print('timed out')
                
        
        self.hmm_nav_server.set_succeeded(result.result)

            

        












        
if __name__=="__main__":
   
    
   
    
    print("hmm nav server available service navigate_hmm")
    s = HMM_navServer()
    rospy.spin()

    