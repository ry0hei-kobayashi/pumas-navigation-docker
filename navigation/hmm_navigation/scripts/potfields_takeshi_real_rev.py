    #!/usr/bin/env python2
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
import tf as tf
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
     lec=np.asarray(data.ranges)
     lec[np.isinf(lec)]=13.5
     
     Fx, Fy,Fth = 0.001,0.001,0
     
     deltaang=4.7124/len(data.ranges)
      
     laserdegs=  np.arange(-2.3562,2.3562,deltaang)
     Fx=0
     Fy = 0.001
     for i,deg in enumerate(laserdegs):
        if  (lec[i]<4.0)  and ( i < 467 ) or ( i >500 ):
            
            Fx = Fx + (1/lec[i])**2 * np.cos(deg)
            Fy = Fy + (1/lec[i])**2 * np.sin(deg)
             
     Fth= np.arctan2(Fy,(Fx+.000000000001))+np.pi
     print('FxFyFth',Fx,Fy,Fth*180/np.pi)
     Fmag= np.linalg.norm((Fx,Fy))
     pose,quat=  listener.lookupTransform('map','base_footprint',rospy.Time(0))
     x,y = pose[0], pose [1]
     euler = euler_from_quaternion(quat)
     th=euler[2]
     xy,xycl=np.array((x,y)) ,   np.array((xcl,ycl))
     euclD=np.linalg.norm(xy-xycl)



     print("xrob,yrob, throbot",x,y,th*180/3.1416)
     print("xclick,yclick",xcl,ycl,"euclD",euclD)
     
     
     Fatrx =( -x + xcl)/euclD
     Fatry =( -y + ycl)/euclD      
     Fatrth=np.arctan2(Fatry, Fatrx) 
     Fatrth=Fatrth-th
     Fmagat= np.linalg.norm((Fatrx,Fatry))
     print ('Fatx, Fatry, Fatrth',Fatrx,Fatry,(Fatrth)*180/np.pi )
     Ftotx= Fmag*np.cos(Fth)*.0012   +    Fmagat*np.cos(Fatrth)
     #Ftotx= Fmag*np.cos(Fth) *600  +    Fmagat*np.cos(Fatrth)
     Ftoty= Fmag*np.sin(Fth)*.0012    +    Fmagat*np.sin(Fatrth)
     #Ftoty= Fmag*np.sin(Fth)  *600   +    Fmagat*np.sin(Fatrth)
     Ftotth=np.arctan2(Ftoty,Ftotx)
     
     if ( Ftotth> np.pi ):
         Ftotth=       -np.pi-    (Ftotth-np.pi)
    
     if (Ftotth < -np.pi):
         Ftotth= (Ftotth     +2 *np.pi)
     
             
     print('Ftotxy',Ftotx,Ftoty,Fmag,Ftotth*180/np.pi)
     rospy.sleep(0.1)
     """Fatmag=np.linalg.norm((Fatrx,Fatry))
     Fmag=np.linalg.norm((Fx,Fy))
     print ("theta robot",th*180/3.1416,'---------------------------')
     print ('fasorFatrth',np.linalg.norm((Fatrx,Fatry)),(Fatrth)*180/3.1416 )
     print ("FXATR,FYATR",Fatrx,Fatry)
     print ('fasorFrepth',np.linalg.norm((Fx,Fy)),Fth*180/3.1416)
     print ("Frepx,Frepy",Fx,Fy)"""
     
        
     """Fx,Fy= Fmag*np.cos(Fth) , Fmag*np.sin(Fth)   
     Fatrx,Fatry= Fatmag*np.cos(Fatrth) , Fatmag*np.sin(Fatrth) """
    
    
     vel=0.07
     if (euclD < 0.5) :
        speed.linear.x=0
        speed.linear.y=0
        speed.angular.z=0

     else:
         if( abs(Ftotth) < .5) :#or (np.linalg.norm((Fx,Fy)) < 100):
             speed.linear.x=  min (current_speed.linear.x+0.015, 0.5)#1.9
             speed.angular.z=0
             print('lin')
         else:
            if Ftotth > -np.pi/2  and Ftotth <0:
                print('Vang-')
                speed.linear.x  = max(current_speed.linear.x -0.003, 0.04)
                speed.angular.z = max(current_speed.angular.z-0.001, -0.2)
            
            if Ftotth < np.pi/2  and Ftotth > 0:
                print('Vang+')
                speed.linear.x  = max(current_speed.linear.x-0.003, 0.04)
                speed.angular.z = min(current_speed.angular.z+0.001,0.2)
            
            
            if Ftotth < -np.pi/2:
                
                print('Vang---')
                speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                speed.angular.z = max(current_speed.angular.z-0.03,-0.5)
            

            if Ftotth > np.pi/2:
                
            
                print('Vang+++')
                speed.linear.x  = max(current_speed.linear.x-0.025, 0.0)
                speed.angular.z = min(current_speed.angular.z+0.03, 0.5)
         """else:
                                        
                                       if Ftotth < 0 :
                                           if (abs( Ftotth ) < np.pi/2):
                                               vel= max(current_speed.linear.z+0.01 ,0.15
                                               print('open curve')
                                        
                                           print('Vang-')
                                           speed.linear.x=vel
                                           speed.angular.z=-0.15
                                       if Ftotth > 0 :
                                           if (abs( Ftotth ) < np.pi/2):
                                               vel= 0.15
                                               print('open curve')
                                        
                                           
                                           print('Vang+')
                                           speed.linear.x=vel
                                           speed.angular.z=.07
                                       if Ftotth < -1.57:
                                           
                                           print('ang--')
                                           speed.linear.x=0
                                           speed.angular.z=-0.3
                                       
                                           
                                       
                                
                                       if Ftotth > 1.57:
                                           print('ang++')
                                           speed.linear.x=0
                                           speed.angular.z=0.3
                           """
        
            
       
        
    

speed=Twist()
speed.angular.z=0
def inoutinout():
    #sub= rospy.Subscriber("/hsrb/odom",Odometry,newOdom)
    global listener,xcl,ycl,current_speed
    rospy.init_node('talker_cmdvel', anonymous=True)
    sub2=rospy.Subscriber("/hsrb/base_scan",LaserScan,readSensor)
    sub3=rospy.Subscriber("/clicked_point",PointStamped,readPoint)
    pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
    listener = tf.TransformListener()
    rate = rospy.Rate(15) # 10hz
    while not rospy.is_shutdown():
        #print (current_speed)
        if (xcl!=0 and ycl!=0):
            pub.publish(speed)
            current_speed=speed
        else:
            current_speed=Twist()

if __name__ == '__main__':
    try:
        inoutinout()
    except rospy.ROSInterruptException:
        pass
