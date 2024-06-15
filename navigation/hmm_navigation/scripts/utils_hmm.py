# -*- coding: utf-8 -*-
"""
Created on Fri Sep 27 16:25:07 2019

@author: oscar
"""
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Twist , PointStamped , Point
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker , MarkerArray
import numpy as np
import pandas as pd


def list_2_markers_array(path, ccxyth):
    xythpath=[]
    marker=Marker() 
    markerarray=MarkerArray()    
    for n in path:
        print(n)
        marker.header.frame_id="/map"
        marker.id=n
        marker.type = marker.SPHERE
        marker.action = marker.ADD
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = ccxyth[n][0]
        marker.pose.position.y = ccxyth[n][1]  
        marker.pose.position.z = 0
        
        markerarray.markers.append(marker)
        marker=Marker() 
        
        
        
    return np.array(markerarray)

def viterbi(obs,Modelo1,PI):
    A, B= Modelo1.A , Modelo1.B    
    delta=np.zeros((len(obs)+1,len(Modelo1.A)))
    phi=np.zeros((len(obs)+1,len(A)))+666
    path =np.zeros(len(obs)+1)
    T=len(obs)
    Modelo1.PI = PI
    delta[0,:]= Modelo1.PI * Modelo1.B[:,obs[0]]
    phi[0,:]=666
    for t in range(len(obs)):
        for j in range(delta.shape[1]):

            delta [t+1,j]=np.max(delta[t] * A[:,j]) * B[j,obs[t]]
            phi[t+1,j]= np.argmax(delta[t] * A[:,j])
    path[T]=int(np.argmax(delta[T,:]))
    for i in np.arange(T-1,0,-1):
        #print (i,phi[i+1,int(path[i+1])])
        path[i]=phi[i+1,int(path[i+1])]
    return(path)

class HMM (object):
             def __init__(self,A,B,PI):
                 self.A=A
                 self.B=B
                 self.PI=PI   
def forw_alg(o_k,Modelo):
    #MATRIX NOTATION
    PI=Modelo.PI
    K= len(o_k)   #Secuencia Observaciones
    N= len(Modelo.A)  #número de estados
    alpha=np.zeros((N,K))
    c_k= np.zeros(K)
    alpha[:,0]= PI
    c_k[0]=1
    for k in range(1,K):
        alpha_k= alpha[:,k-1]
        a= Modelo.A[:,:]
        b= Modelo.B[:,o_k[k]]
        alpha[:,k]=(b*np.dot(alpha_k,a))#* c_k[k-1]
        #c_k[k]=1/alpha[:,k].sum()
    return alpha #,c_k
def  backw_alg(o_k,Modelo):
    #MATRIX NOTATION

    PI=Modelo.PI
    K= len(o_k)   #Secuencia Observaciones
    N= len(Modelo.A)  #número de estados
    beta=np.zeros((N,K))
    beta[:,-1]=1
    for t in range(K-2,-1,-1):
        beta_t1=beta[:,t+1]
        beta_t1
        a= Modelo.A[:,:]
        b= Modelo.B[:,o_k[t]]
        beta[:,t]= b*np.dot(a,beta_t1)
    return beta
    
    