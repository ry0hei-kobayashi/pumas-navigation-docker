import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ObstacleDetectorNode:
    def __init__(self):
        self.bridge = CvBridge()
        self.depth_image = None
        self.color_image = None
        
        # Subscribe to the depth image and color image topics
        
        # rospy.Subscriber("/hsrb/head

#!/usr/bin/env python3
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge, CvBridgeError

class ObstacleDetector:
    def __init__(self):
        self.bridge = CvBridge()
        rospy.Subscriber("/hsrb/head_rgbd_sensor/depth_registered/image_rect_raw", Image, self.depth_callback)
        # self.obstacle_pub = rospy.Publisher("obstacle_detection", String, queue_size=10)

    def depth_callback(self, data):
        try:
            depth_image = self.bridge.imgmsg_to_cv2(data, "32FC1")
        except CvBridgeError as e:
            print(e)
        print(depth_image)
        min_distance = np.min(depth_image)
        if min_distance < 0.5:
            # self.obstacle_pub.publish("STOP")
            print('stop')
        else:
            # self.obstacle_pub.publish("GO")
            print('go')

if __name__ == '__main__':
    rospy.init_node('obstacle_detector')
    obstacle_detector = ObstacleDetector()
    rospy.spin()
