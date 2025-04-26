#!/usr/bin/python3

import numpy as np
import struct

import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

import rospy
from rospy.core import is_shutdown
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointField
from detic_ros.srv import DeticDetection3DService
from detic_ros.msg import DeticDetection3DArray
from nav_msgs.msg import OccupancyGrid

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

import tf.transformations as tft

class CollitionFromDetic():
    def __init__(self): 
        #self.detic_detection = rospy.ServiceProxy('/detic_detection_3d_service', DeticDetection3DService)
        self.detic_detection = rospy.Subscriber('/detic_detection_3d', DeticDetection3DArray, self.detic_cb)
        self.map_callback = rospy.Subscriber('/map' ,OccupancyGrid, self.map_callback)
        self.map_pub = rospy.Publisher('/collision_map_from_detic', OccupancyGrid, queue_size=1, latch=True)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def map_callback(self, msg):
        self.original_map = msg

    def detic_cb(self, msg):
        if self.original_map is None:
            return

        self.projection_collision2map(msg)

    def projection_collision2map(self, msg):
        rospy.logwarn('DeticDetection3DServiceClient -> waiting for start detic service server')
        #rospy.wait_for_service('detic_detection_3d_service')
        detections_list = msg.detected_object_array

        try:
            #response = self.detic_detection()
            #detected_objs = response.detections
            #detections_list = detected_objs.detected_object_array

            map_with_collision = OccupancyGrid()
            map_with_collision.header.frame_id = self.original_map.header.frame_id
            map_with_collision.info = self.original_map.info
            map_with_collision.data = list(self.original_map.data)

            trans = self.tf_buffer.lookup_transform("map", "head_rgbd_sensor_rgb_frame", msg.header.stamp, rospy.Duration(1.0))
            translation = (
                    trans.transform.translation.x,
                    trans.transform.translation.y,
                    trans.transform.translation.z,
                    )
            rotation = (
                    trans.transform.rotation.x,
                    trans.transform.rotation.y,
                    trans.transform.rotation.z,
                    trans.transform.rotation.w,
                    )
            T = tft.concatenate_matrices(tft.translation_matrix(translation), tft.quaternion_matrix(rotation)) #4x4

    
            for detection in detections_list:
                pcl_data = detection.point_cloud
                points_list = []
    
                for point in point_cloud2.read_points(pcl_data, field_names=('x', 'y', 'z', 'rgb')):
                    x, y, z, rgb = point
                    #rgb_int = struct.unpack('I', struct.pack('f', rgb))[0]
    
                    #r = (rgb_int >> 16) & 0x0000ff
                    #g = (rgb_int >> 8) & 0x0000ff
                    #b = rgb_int & 0x0000ff
    
                    #points_list.append([x, y, z, r, g, b])
                    points_list.append([x, y, z])
    
                points = np.array(points_list)

                if points.shape[0] == 0:
                    continue # if no points

                points_hom = np.hstack((points[:, :3], np.ones((points.shape[0], 1))))
                points_map = (T @ points_hom.T).T[:, :3]
 
                for map_x, map_y, map_z in points_map:

                    grid_x = int((map_x - map_with_collision.info.origin.position.x) / map_with_collision.info.resolution)
                    grid_y = int((map_y - map_with_collision.info.origin.position.y) / map_with_collision.info.resolution)
                    
                    if 0 <= grid_x < map_with_collision.info.width and 0 <= grid_y < map_with_collision.info.height:
                        index = grid_y * map_with_collision.info.width + grid_x
                        map_with_collision.data[index] = 100

            map_with_collision.header.stamp = rospy.Time.now()
            self.map_pub.publish(map_with_collision)
    
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s" % e)

if __name__ == "__main__":
    rospy.init_node('detic_client')
    cfd = CollitionFromDetic()
    rospy.spin()
