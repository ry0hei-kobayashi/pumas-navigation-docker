#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""
from geometry_msgs.msg import Quaternion , Point
import numpy as np
import rospy
import message_filters
import cv2
from std_msgs.msg import String ,ColorRGBA
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseWithCovarianceStamped 
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker , MarkerArray
from tf.transformations import *
from tf.msg import tfMessage
from tf.transformations import euler_from_quaternion
from utils_hmm import  viterbi
from utils_hmm import forw_alg
from utils_hmm import backw_alg
from joblib import dump, load
import matplotlib.pyplot as plt
import math

odom_adjust,odom_adjust_aff=np.zeros(3),np.zeros(3)
first_adjust= np.zeros(3)
first_adjust= np.zeros(3)
xyth_hmm1=np.zeros(3)
xyth_hmm2=np.zeros(3)
xyth_dual=np.zeros(3)
hmm12real=np.zeros(3)
xyth_odom_prueba=np.zeros(3)
first=True
first_aff =True
first_adjust_aff=True
first_adjust_dual =True
last_states=[]
last_states_2=[]
last_states_real=[]
delta_xyth=[]
o_k=[]
o_k2=[]
#transitions= np.load('trans.npy')
buf_vit=60
clf=load('aff_prop_class.joblib')
marker=Marker()   
markerarray=MarkerArray()
first_adjust_2=True
last_states_trans=[0,0]
last_states_trans_2=[0,0]
last_states_trans_real=[0,0]
centroides = np.load('ccvk.npy')
ccxyth= np.load('ccxyth.npy')





class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI  

#############
A, B, PI= np.load('A.npy') , np.load('B.npy') , np.load('PI.npy')
Modelo1= HMM(A,B,PI)
A2, B2, PI2= np.load('A.npy') , np.load('B2.npy') , np.load('PI2.npy')## SAME MATRIX A BUT COULD NOT BE
Modelo2= HMM(A,B2,PI2)



    

def callback(laser,pose,odom):
        global xyth , xyth_odom , xyth_hmm1,xyth_hmm2, xyth_dual, hmm12real , xyth_odom_prueba , ccxyth , centroides , A
        global first,first_aff , last_states_trans , last_states_trans_2 ,last_states_trans_real
        global odom_adjust,odom_adjust_aff,first_adjust , first_adjust_2 , first_adjust_aff,first_adjust_dual
        n= len(ccxyth)
        markerarray=MarkerArray()
        ###PUB HMM
        ##EDGES
        ss= np.arange(len(A))
        start_point = Point()   
        mid_point=Point()     #start point
        end_mid_point=Point()
        end_point = Point()        #end point
        markerarray=MarkerArray()
        line_color = ColorRGBA()       # a nice color for my line (royalblue)
        line_color.r = 0.254902
        line_color.g = 0.411765
        line_color.b = 0.882353
        line_color.a = 1.0
        num_markers=0
        """for s1 in ss:
            for s2 in ss:
                    if (s1!=s2)and (A[s1,s2]!=0):# and (np.linalg.norm(ccxyth[s1,:2]-ccxyth[s2,:2])<1)   :#and(s1==s or s2==s)
                        num_markers+=1
                        start_point.x = ccxyth[s1,0]
                        start_point.y = ccxyth[s1,1]
                        start_point.z = 0.2
                        
        
                        end_point.x = ccxyth[s2,0]
                        end_point.y = ccxyth[s2,1]
                        end_point.z = 0.2
                        marker3 = Marker()
                        marker3.id = num_markers
                        marker3.header.frame_id = '/map'
                        marker3.type = Marker.LINE_LIST
                        marker3.ns = 'Testline'
                        marker3.action = Marker.ADD
                        marker3.scale.x = 0.015
                        marker3.points.append(start_point)
                        marker3.points.append(end_point)
                        marker3.colors.append(line_color)
                        marker3.colors.append(line_color)
                        markerarray.markers.append(marker3)"""

        ss= np.arange(len(A))
        start_point = Point()   
        end_point = Point()     
        markerarray=MarkerArray()

        line_color = ColorRGBA()       # a nice color for my line (royalblue)
        line_color.r = 0.254902
        line_color.g = 0.411765
        line_color.b = 0.882353
        line_color.a = 1.0

        mark_count=0
        marker3 = Marker()
        marker3.id = 0
        marker3.header.frame_id = 'map'
        marker3.type = Marker.LINE_LIST#STRIP
        marker3.ns = 'edges'
        marker3.action = Marker.ADD
        marker3.scale.x = 0.015
        #Topological map
        
        for s1 in ss:
            for s2 in ss:
                if (s1!=s2)and (A[s1,s2]!=0):# and (np.linalg.norm(ccxyth[s1,:2]-ccxyth[s2,:2])<1)   :#and(s1==s or s2==s)
                    mark_count+=1
                    start_point.x = ccxyth[s1,0]
                    start_point.y = ccxyth[s1,1]
                    start_point.z = 0.2
                    end_point.x = ccxyth[s2,0]
                    end_point.y = ccxyth[s2,1]
                    end_point.z = 0.2
                    marker3.colors.append(line_color)
                    marker3.colors.append(line_color)
                    marker3.points.append(start_point)
                    marker3.points.append(end_point)
                    markerarray.markers.append(marker3)
                    start_point,end_point = Point(),Point()

                                
       

        
        ############NODES
        for n in range(len(ccxyth)):
           
            quaternion = quaternion_from_euler(0, 0, ccxyth[(int)(n)][2])
            marker=Marker()
            marker.header.frame_id="map"
            marker.header.stamp = rospy.Time.now()
            marker.id=n
            marker.type = marker.ARROW
            marker.action = marker.ADD
            marker.scale.x = 0.1
            marker.scale.y = 0.01
            marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0
            marker.pose.orientation.x = quaternion[0]
            marker.pose.orientation.y = quaternion[1]
            marker.pose.orientation.z = quaternion[2]        
            marker.pose.orientation.w = quaternion[3]
            marker.pose.position.x = ccxyth[(int)(n)][0]
            marker.pose.position.y = ccxyth[(int)(n)][1]  
            marker.pose.position.z = 0
            marker.lifetime.nsecs=1
            markerarray.markers.append(marker)

        pub2.publish(markerarray)
 
       
        
        #GET REAL ROBOT POSE 
        text_to_rviz=""
        x_odom=odom.pose.pose.position.x
        y_odom=odom.pose.pose.position.y
        quaternion = (
        odom.pose.pose.orientation.x,
        odom.pose.pose.orientation.y,
        odom.pose.pose.orientation.z,
        odom.pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        th_odom=euler[2]
    
       
        
        
        
        
        lec=np.asarray(laser.ranges)
        
        
        #GET BOTH O_K 's
        #print (lec.shape)
        lec[np.isinf(lec)]=13.5
        lec=np.clip(lec,0,5)
        lec_str=str(lec[0])+','
        for i in range (1,len(lec)):
            lec_str=lec_str+str(lec[i])+',' 
            #print (lec_str)
        
        
        
        
        


        Vk_aff= (int)( clf.predict(lec.reshape(1,-1)))
        
       
        
#        
        quaternion = (
        pose.pose.orientation.x,
        pose.pose.orientation.y,
        pose.pose.orientation.z,
        pose.pose.orientation.w)
        euler = euler_from_quaternion(quaternion)
        #roll = euler[0]
        #pitch = euler[1]
        #yaw = euler[2]
        symbol= np.power(lec.T-centroides,2).sum(axis=1,keepdims=True).argmin()
        symbol2= Vk_aff
        if len(o_k) >=buf_vit:
            o_k.pop(0)
        o_k.append(symbol)
        if len(o_k2) >=buf_vit:
            o_k2.pop(0)
        o_k2.append(symbol2)
        xyth= np.asarray((pose.pose.position.x,pose.pose.position.y,euler[2]))
        xyth_odom=np.asarray((x_odom, y_odom,th_odom))
        #delta_xyth.append(xyth)
        delta_xyth.append(xyth_odom)
        if (len(delta_xyth)>2):
            delta_xyth.pop(0)
            delta= delta_xyth[1]-delta_xyth[0]
            delta_phase=math.atan2(delta[0],delta[1])
            delta_mag=np.linalg.norm(delta[:2])
            print('xyth[-1]-xyth_odom[-1]',xyth[-1]-xyth_odom[-1])
            delta_phase_rotated=delta_phase  - (xyth_hmm1[-1]-xyth_odom[-1])
            deltarotated=np.asarray( (delta_mag*math.sin(delta_phase_rotated),delta_mag*math.cos(delta_phase_rotated) , delta[2]   ) )
            xyth_hmm1+=deltarotated
            xyth_hmm2+=deltarotated
            xyth_dual+=deltarotated

            xyth_odom_prueba+=delta 
            


        #QUANTIZING READS (CLASIFIYNG ok2)
       
        xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
        xyth_odomcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))

        

        if (len(o_k)< buf_vit):
            
            print ( "FILLING BUFFER HMM1 and HMM2")
        
        if (len(o_k)>= buf_vit) and (len(o_k2)>= buf_vit):
        
            
            vit_est= viterbi(o_k,Modelo1,Modelo1.PI)
            last_states.append((int)(vit_est[-1]))
            vit_est_2= viterbi(o_k2[-10:],Modelo2,Modelo2.PI)
            last_states_2.append((int)(vit_est_2[-1]))
            last_states_real.append(xyth_odomcuant)
            

            

            if first:
            #if (len (last_states)<10 and first and len (last_states_2)<10)  :
                if (last_states[-1]==xyth_odomcuant) :#and (last_states_2[-1]==xyth_odomcuant):
                    print ("READJUST ODOM Hmm1 , first adjust to centroid ",xyth_odom,"to",ccxyth[last_states[-1]]  )
                    xyth_hmm1= ccxyth[last_states[-1]]
                    xyth_hmm1[2]=xyth[2]# BRUJULA
                    #last_states_trans[1]=last_states[-1]
                    first= False
                    #rospy.sleep(1)
              

            if first_aff:
            #if (len (last_states)<10 and first and len (last_states_2)<10)  :
                if (last_states_2[-1]==xyth_odomcuant) :#and (last_states_2[-1]==xyth_odomcuant):
                    print ("READJUST ODOM Hmm2, first adjust to centroid ",xyth_odom,"to",ccxyth[last_states[-1]]  )
                    xyth_hmm2= ccxyth[last_states_2[-1]]
                    xyth_hmm2[2]=xyth[2]# BRUJULA
                    #last_states_trans[1]=last_states[-1]
                    first_aff= False
                    #rospy.sleep(1)
             









                
            if (len (last_states)>=10 ) and (len (last_states_2)>=10) and (len(last_states_real)>=10)  :
                last_states.pop(0)
                last_states_2.pop(0)
                last_states_real.pop(0)
                
               
                
                    

                   

                if (last_states[-1]!=last_states[-2]):
                    last_states_trans=last_states[-2:]
                    first_adjust_2=True

                if (last_states_2[-1]!=last_states_2[-2]):
                    last_states_trans_2=last_states_2[-2:]
                    first_adjust_aff=True
                    first_adjust_dual=True
                

                if (last_states_real[-1]!=last_states_real[-2]):
                    last_states_trans_real=last_states_real[-2:]
                




                if(last_states[-1]==xyth_odomcuant) : #and (last_states_trans_real[-2]==last_states_trans[-2])  :

                    #print('last states',last_states)
                    if  first_adjust_2 :
                        
                        print(' Correct TRANSITION FIRST TIME STATE  DETECTED hmm1 TRUSTED STATE  \n \n \n \n  ' )
                        print ("TRANSITION FROM",last_states_trans )
                        #print ("Origin set to ",transitions[last_states_trans[0],last_states_trans[1]])
                        print ('xyth_hmm1',xyth_hmm1)
                        first_adjust_2=False
                       
                                                 
                            
                        
                        xyth_hmm1= xyth#ccxyth[last_states[-1]]#xyth#transitions[last_states_trans[0],last_states_trans[1]]
                        xyth_hmm1[2]=xyth[2]
                        
                        #rospy.sleep(.11)
                


                if(last_states_2[-1]==xyth_odomcuant):#  and (last_states_trans_real[-2]==last_states_trans_2[-2])  :
                    if  first_adjust_aff :
                        
                        print(' Correct TRANSITION FIRST TIME STATE  DETECTED hmm2  TRUSTED STATE  \n \n \n \n  ' )
                        print ("TRANSITION FROM",last_states_trans_2 )
                        #print ("Origin set to ",transitions[last_states_trans[0],last_states_trans[1]])
                        print ('xyth_hmm2',xyth_hmm2)
                        first_adjust_aff=False
                      
                                                 
                            
                        
                        xyth_hmm2= ccxyth[last_states_2[-1]] #xyth  #ccxyth[last_states_2[-1]]#xyth#transitions[last_states_trans[0],last_states_trans[1]]
                        xyth_hmm2[2]=xyth[2]
                        
                        #rospy.sleep(.11)
                if(last_states_2[-1]==xyth_odomcuant)   and (last_states[-1]==last_states_2[-1])  :
                    print(' Correct TRANSITION FIRST TIME STATE  DETECTED DUAL  \n \n \n \n  ' )

                    if  first_adjust_dual :
                        
                        
                        print ("TRANSITION FROM",last_states_trans_2 )
                        #print ("Origin set to ",transitions[last_states_trans[0],last_states_trans[1]])
                        print ('xyth_hmm2',xyth_dual)
                        #first_adjust_dual=False
                      
                                                 
                            
                        
                        xyth_dual= xyth#ccxyth[last_states[-1]]#xyth#transitions[last_states_trans[0],last_states_trans[1]]
                        xyth_dual[2]=xyth[2]
                        
                        #rospy.sleep(.11)
            
            
            print ('Most likely state seq given O',vit_est[-5:])
            print ('Most likely states given O Modelo aff prop (',vit_est_2[-5:])
           
        
        print('lec vk_aff'+str(Vk_aff)+'lec vk'+str(symbol)  +') , ccxyth[ '+str(xythcuant)+']='+str(ccxyth[xythcuant]) + '\n')
        print ('Pose',xyth)
        print('Wheel ' ,xyth_odom)
        print(' HMM 1' ,xyth_hmm1)
        print(' HMM 2' ,xyth_hmm2)
        print(' HMM DUAL' ,xyth_dual)
        

        """with  open('dataset_hokuyo_wrs/odometry_and_odometrycorrected.txt' , 'a') as out:
                                    text_line=""
                                    for value in xyth:
                                        text_line+=str(value)+','
                                    for value in xyth_odom:
                                        text_line+=str(value)+','
                                    for value in xyth_hmm1:
                                        text_line+=str(value)+','
                                    for value in xyth_hmm2:
                                        text_line+=str(value)+','
                                    for value in xyth_dual:
                                        text_line+=str(value)+','
                                    text_line+='\n'
                                    out.write(text_line)
                                
                                text_to_rviz+= 'REAL Pose'+ str(xyth)+'\n'+'Wheel odom  '+ str(xyth_odom)+'\n'+'Dual HMM 1'+ str(xyth_hmm1)+'\n'+'Dual HMM 2'+ str(xyth_hmm2)+'\n'
                        """

        
        




        markerarray=MarkerArray()
        marker= Marker()
        marker.header.frame_id="map"
        marker.header.stamp = rospy.Time.now()
        marker.id=1

        
        marker.scale.z = 0.2
        marker.pose.position.x = xyth[0]+0.5
        marker.pose.position.y = xyth[1]+0.5  
        marker.pose.position.z = 0
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.type = marker.TEXT_VIEW_FACING
        marker.action = marker.ADD
        marker.text=text_to_rviz
        markerarray.markers.append(marker)
        pub.publish(markerarray)

        
       

        """    with  open('dataset_candidatura_wr/estimadores.txt' , 'a') as out:
                            text_line=""
                            for value in xyth_odom:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust:
                                text_line+=str(value)+','
                            for value in xyth_odom-odom_adjust_aff:
                                text_line+=str(value)+','
                            for value in xyth:
                                text_line+=str(value)+','        
                            text_line+= str(last_states[-1])+','+str(last_states[-1])+','
                            text_line+='\n'
                            out.write(text_line)
                    """    
	
def listener():
    
    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    #clf=load('aff_prop_class.joblib')
    global pub , pub2
    rospy.init_node('listener', anonymous=True)
    #twist = message_filters.Subscriber('cmd_vel',Twist)
    symbol= message_filters.Subscriber('/hsrb/base_scan',LaserScan)
    pub= rospy.Publisher('aa/Viterbi',MarkerArray,queue_size=1)
    pub2 = rospy.Publisher('/aa/HMM_topo/', MarkerArray, queue_size=1)  
    #pose  = message_filters.Subscriber('/navigation/localization/amcl_pose',PoseWithCovarianceStamped)#TAKESHI REAL
    pose  = message_filters.Subscriber('/hsrb/base_pose',PoseStamped)#TAKESHI GAZEBO
    odom= message_filters.Subscriber("/hsrb/wheel_odom",Odometry)
    #ats= message_filters.ApproximateTimeSynchronizer([symbol,odom,twist],queue_size=5,slop=.1,allow_headerless=True)
    ats= message_filters.ApproximateTimeSynchronizer([symbol,pose,odom],queue_size=5,slop=.1,allow_headerless=True)
    ats.registerCallback(callback)
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    
    
    
   
    listener()
