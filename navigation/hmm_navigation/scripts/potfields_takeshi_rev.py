#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:54:18 2019

@author: oscar
"""

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PointStamped
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
import numpy as np
xcl,ycl=0,0

def normalize(x,y):
    xn= x/np.linalg.norm((x,y))
    yn= y/np.linalg.norm((x,y))
    return ((xn,yn))

def newOdom (msg):
    global x
    global y
    global th
    
    x=msg.pose.pose.position.x
    y=msg.pose.pose.position.y
    quaternion = (
    msg.pose.pose.orientation.x,
    msg.pose.pose.orientation.y,
    msg.pose.pose.orientation.z,
    msg.pose.pose.orientation.w)
    euler = euler_from_quaternion(quaternion)
    th=euler[2]
def readPoint(punto):
     global xcl
     global ycl
     xcl =punto.point.x
     ycl =punto.point.y
           
def readSensor(data):
     global xcl
     global ycl
     lec=np.asarray(data.ranges)
     lec[np.isinf(lec)]=13.5
     
     Fx, Fy,Fth = 0.001,0.001,0
     
     deltaang=4.7124/len(data.ranges)
      
     laserdegs=  np.arange(-2.3562,2.3562,deltaang)
     Fx=0
     Fy = 0.001
     for i,deg in enumerate(laserdegs):
         
         if (lec[i] <2.61):
             Fx = Fx + (1/lec[i])**2 * np.cos(deg)
             Fy = Fy + (1/lec[i])**2 * np.sin(deg)
             
     Fth= np.arctan2(Fy,(Fx+.000000000001))+np.pi
     print('FxFyFth',Fx,Fy,Fth*180/np.pi)
     Fmag= np.linalg.norm((Fx,Fy))
   
     xy,xycl=np.array((x,y)) ,   np.array((xcl,ycl))
     euclD=np.linalg.norm(xy-xycl)
     if euclD < 0.3:
        xcl=0
        ycl=0
     print("xrob,yrob, throbot",x,y,th*180/3.1416)
     print("xclick,yclick",xcl,ycl,"euclD",euclD)
     
     
     Fatrx =( -x + xcl)/euclD
     Fatry =( -y + ycl)/euclD      
     Fatrth=np.arctan2(Fatry, Fatrx) 
     Fatrth=Fatrth-th
     Fmagat= np.linalg.norm((Fatrx,Fatry))
     print ('Fatx, Fatry, Fatrth',Fatrx,Fatry,(Fatrth)*180/np.pi )
     Ftotx= Fmag*np.cos(Fth)*.00251   +    Fmagat*np.cos(Fatrth)
     Ftoty= Fmag*np.sin(Fth)*.00251    +    Fmagat*np.sin(Fatrth)
     Ftotth=np.arctan2(Ftoty,Ftotx)
     
     if ( Ftotth> np.pi ):
         Ftotth=       -np.pi-    (Ftotth-np.pi)
    
     if (Ftotth < -np.pi):
         Ftotth= (Ftotth     +2 *np.pi)
     
             
     print('Current Speed',current_speed)
     print('Ftotxy',Ftotx,Ftoty,Ftotth*180/np.pi)
     Fatmag=np.linalg.norm((Fatrx,Fatry))
     Fmag=np.linalg.norm((Fx,Fy))
     print ("theta robot",th*180/3.1416,'---------------------------')
     print ('fasorFatrth',np.linalg.norm((Fatrx,Fatry)),(Fatrth)*180/3.1416 )
     print ("FXATR,FYATR",Fatrx,Fatry)
     print ('fasorFrepth',np.linalg.norm((Fx,Fy)),Fth*180/3.1416)
     print ("Frepx,Frepy",Fx,Fy)
     
        
     """Fx,Fy= Fmag*np.cos(Fth) , Fmag*np.sin(Fth)   
     Fatrx,Fatry= Fatmag*np.cos(Fatrth) , Fatmag*np.sin(Fatrth) """
    
    
     vel=0 
     if( abs(Ftotth) < .1) :#or (np.linalg.norm((Fx,Fy)) < 100):
         #speed.linear.x=1.9
         speed.linear.x= min(current_speed.linear.x+0.1 ,3.0)
         print('lin')
         speed.angular.z=0
     
     else:
         
        if Ftotth <  np.pi/2:
            if (abs( Ftotth ) ):
                vel=.24
                print('open curve')
         
            print('Vang-')
            speed.linear.x=current_speed.linear.x
            speed.angular.z=current_speed.linear.z-0.15
        if Ftotth > 0:
            if (abs( Ftotth ) < np.pi/2):
                speed.linear.x=current_speed.linear.x
                speed.angular.z=current_speed.linear.z-0.15
                print('open curve')
         
            
            
        """if Ftotth < -1.57:
            
            print('ang++')
            speed.linear.x=0
            speed.angular.z=0.5
        
            
        
 
        if Ftotth > 1.57:
            print('ang-')
            speed.linear.x=0
            speed.angular.z=-0.5"""

        
            
       
        
    

speed=Twist()
speed.angular.z=0
def inoutinout():
    global xcl,ycl,current_speed
    sub= rospy.Subscriber("/hsrb/odom",Odometry,newOdom)
    sub2=rospy.Subscriber("/hsrb/base_scan",LaserScan,readSensor)
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    rospy.init_node('talker_cmdvel', anonymous=True)
    rate = rospy.Rate(15) # 10hz
    while not rospy.is_shutdown():
        hello_str = "Time %s" % rospy.get_time()
        
        if (xcl!=0 and ycl!=0):
            pub.publish(speed)
            current_speed=speed


        else:
            current_speed=0

if __name__ == '__main__':
    try:
        inoutinout()
    except rospy.ROSInterruptException:
        pass
