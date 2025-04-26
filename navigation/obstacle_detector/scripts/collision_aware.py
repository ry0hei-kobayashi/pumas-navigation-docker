import numpy as np
import matplotlib.pyplot as plt
import rospy
from nav_msgs.msg import OccupancyGrid

#要件：slamのときのみtrue, obs_detectionがtrueになったらdynamic_obstacle_mapのsrvを投げる，障害物登録する．

class CollisionAware():

    def __init__(self):
        self.sub_dynamic_obs_map = rospy.Subscriber("/dynamic_obstacle_map", OccupancyGrid ,self.dynamic_obs_map_cb)

    def dynamic_obs_map_cb(self, msg):
        width = msg.info.width
        height = msg.info.height
        data = np.array(msg.data).reshape((height, width))

        occupied = (data==100).astype(np.uint8)

        plt.imshow(occupied, cmap='gray')
        plt.pause(0.001)
        plt.clf()

if __name__ == '__main__':
    plt.ion()
    rospy.init_node("collision_aware_node")
    ca = CollisionAware()
    rospy.spin()
