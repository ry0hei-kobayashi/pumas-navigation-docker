import tf
import tf2_ros
import numpy as np
import rospy
import actionlib
import rospkg
from hmm_navigation.msg import NavigateAction ,NavigateActionGoal,NavigateActionFeedback,NavigateActionResult
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PointStamped
from visualization_msgs.msg import Marker , MarkerArray


##################################################

def quantized(xyth,ccxyth):
    xythcuant=np.argmin(np.linalg.norm(xyth-ccxyth,axis=1))
    x,y=ccxyth[xythcuant,:2]
    return ((x,y),(xythcuant))
class grafo (object):
             def __init__(self,nodos,conec):
                 self.nodos=nodos
                 self.conec=conec        

def dijkstra(nodoinicial,nodofinal,graphe):
    

    numnodos= len(graphe.nodos)
    con = graphe.conec
    D= np.ones(numnodos)*np.inf
    Prv= np.ones(numnodos)*np.inf
    V= np.zeros(numnodos)
    a = nodoinicial
    D[a]=0
    Prv[a]=0
    Prv[np.where(con[a]!=np.inf)]=a
    V[a]=1
    Dacc=D[a]
    ########
    D=np.minimum(D,con[a]+D[a])
    cont=0
    sucess=False
    while(sucess==False):
        a = np.argmin(D+np.where (V==1,np.inf, V))
        Dacc=D[a]
        Prv[np.where(D>(con[a]+Dacc) )]=a
        V[a]=1
        D=np.minimum(D,con[a]+Dacc)
        if (a== nodofinal):
            print("RUTA CALCULADA ")
            sucess=True
    rutainv=[]
    rutainv.append(nodofinal)
    while(rutainv[-1]!=nodoinicial):
        prv=Prv[int(rutainv[-1])]
        rutainv.append(prv)

    ruta=[]
    for n in reversed(rutainv):
        ruta.append((int)(n))
    return(ruta)


def Markov_A_2_grafo(A,ccxyth):
    dists=np.zeros(A.shape)
    for i in range(A.shape[0]):
        for j in range (A.shape[1]):
            if A[i,j]!=0 :
                dists[i,j]= np.linalg.norm(ccxyth[i]-ccxyth[j])    
    
    
    con = np.where(dists==0,np.inf,dists)
    graphe2=grafo(ccxyth,con)
    return graphe2

def pose2feedback(pose_robot,quat_robot,timeleft,euclD):
    feed = NavigateActionFeedback()
    feed.feedback.x_robot   = pose_robot[0]
    feed.feedback.y_robot   = pose_robot[1]
    euler= tf.transformations.euler_from_quaternion((quat_robot[0] ,quat_robot[1] ,quat_robot[2] ,quat_robot[3] )) 
    feed.feedback.yaw_robot = euler[2]
    feed.feedback.timeleft    = timeleft
    feed.feedback.euclD= euclD

    return feed  





#########################################################
global listener , pub ,pub2, pub3 , pub_goal , ccxyth , A
###########################################################
rospy.init_node('hmm_navigation_actionlib_server')
pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=1)
pub2 = rospy.Publisher('/aa/Markov_NXT/', PointStamped, queue_size=1)  
pub3= rospy.Publisher('aa/Markov_route',MarkerArray,queue_size=1)
pub_goal= rospy.Publisher('/clicked_point',PointStamped,queue_size=1)

rospack = rospkg.RosPack()
listener = tf.TransformListener()

file_path_A = rospack.get_path('hmm_navigation')  + '/scripts/hmm_nav_lab/A.npy'   # Transition Matrix for HMM ( or any other 2D pose centroids (x,y,theta))
A=np.load(file_path_A)

file_path_ccxyth = rospack.get_path('hmm_navigation')  + '/scripts/hmm_nav_lab/ccxyth.npy'  #Observation Symbols centroids ##
ccxyth=np.load(file_path_ccxyth)



