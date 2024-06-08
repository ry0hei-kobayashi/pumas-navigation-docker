#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from sensor_msgs import point_cloud2
from cv_bridge import CvBridge, CvBridgeError
class DepthToPointCloud:
    def __init__(self):
        self.bridge = CvBridge()
        # サブスクライバ
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.camera_info_sub = rospy.Subscriber('/camera/depth/camera_info', CameraInfo, self.camera_info_callback)
        # パブリッシャ
        self.pc_pub = rospy.Publisher('/camera/depth/points', PointCloud2, queue_size=1)
        self.camera_info_received = False
    def camera_info_callback(self, msg):
        self.fx = msg.K[0]
        self.fy = msg.K[4]
        self.cx = msg.K[2]
        self.cy = msg.K[5]
        self.camera_info_received = True
    def depth_callback(self, data):
        if not self.camera_info_received:
            return
        try:
            # Depth image to numpy array
            depth_image = self.bridge.imgmsg_to_cv2(data, desired_encoding="passthrough")
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        # Get depth image dimensions
        height, width = depth_image.shape
        # Generate grid of (u, v) coordinates
        u = np.arange(width)
        v = np.arange(height)
        u, v = np.meshgrid(u, v)
        # Calculate z, x, y coordinates
        z = depth_image / 1000.0  # depth value scaling (assuming depth in millimeters)
        x = (u - self.cx) * z / self.fx
        y = (v - self.cy) * z / self.fy
        # Filter out points with z == 0
        mask = z > 0
        x = x[mask]
        y = y[mask]
        z = z[mask]
        # Stack x, y, z coordinates into a single array
        points = np.stack((x, y, z), axis=-1)
        # Convert to PointCloud2 message
        header = data.header
        point_cloud = point_cloud2.create_cloud_xyz32(header, points.reshape(-1, 3))
        # Publish the point cloud
        self.pc_pub.publish(point_cloud)
if __name__ == '__main__':
    rospy.init_node('depth_to_pointcloud')
    dtp = DepthToPointCloud()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass








