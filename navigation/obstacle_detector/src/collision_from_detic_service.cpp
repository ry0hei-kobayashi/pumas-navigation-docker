#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <detic_ros/DeticDetection3DArray.h>
#include <detic_ros/DeticDetection3D.h>
#include <detic_ros/DeticDetection3DService.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

class CollisionFromDetic
{
private:
    ros::NodeHandle nh_;
    ros::ServiceClient detic_client_;
    ros::Subscriber map_sub_;
    ros::Subscriber risk_sub_;
    ros::Publisher map_pub_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;

    nav_msgs::OccupancyGrid::ConstPtr latest_map_;
    bool collision_risk = false;

public:
    CollisionFromDetic() : tf_listener_(tf_buffer_)
    {
        detic_client_ = nh_.serviceClient<detic_ros::DeticDetection3DService>("/detic_detection_3d_service");
        map_sub_ = nh_.subscribe("/map", 1, &CollisionFromDetic::mapCallback, this);
        risk_sub_ = nh_.subscribe("/navigation/obs_detector/collision_risk", 1, &CollisionFromDetic::riskCallback, this);
        map_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("/collision_map_from_detic", 1, true);

        ros::Timer timer = nh_.createTimer(ros::Duration(0.5), &CollisionFromDetic::timerCallback, this);
    }

    void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
    {
        latest_map_ = msg;
    }

    void riskCallback(const std_msgs::Bool::ConstPtr& msg)
    {
        collision_risk = msg->data;
    }

    void timerCallback(const ros::TimerEvent&)
    {
        if (!latest_map_)
        {
            ROS_WARN_THROTTLE(5.0, "Waiting for map...");
            return;
        }

        if (!collision_risk)
        {
            return;
        }

        detic_ros::DeticDetection3DService srv;

        if (!detic_client_.call(srv))
        {
            ROS_WARN("Failed to call DeticDetection3DService");
            return;
        }

        projectCollisionToMap(srv.response.detections);
    }

    void projectCollisionToMap(const detic_ros::DeticDetection3DArray& detections)
    {
        nav_msgs::OccupancyGrid collision_map = *latest_map_;

        try
        {
            geometry_msgs::TransformStamped transform_stamped = tf_buffer_.lookupTransform(
                "map", "head_rgbd_sensor_rgb_frame", detections.header.stamp, ros::Duration(1.0));

            Eigen::Translation3d translation(
                transform_stamped.transform.translation.x,
                transform_stamped.transform.translation.y,
                transform_stamped.transform.translation.z
            );

            Eigen::Quaterniond rotation(
                transform_stamped.transform.rotation.w,
                transform_stamped.transform.rotation.x,
                transform_stamped.transform.rotation.y,
                transform_stamped.transform.rotation.z
            );

            Eigen::Affine3d T = translation * rotation;

            for (const auto& detection : detections.detected_object_array)
            {
                const sensor_msgs::PointCloud2& pcl_msg = detection.point_cloud;

                pcl::PointCloud<pcl::PointXYZRGB> cloud;
                pcl::fromROSMsg(pcl_msg, cloud);

                #pragma omp parallel for
                for (int i = 0; i < static_cast<int>(cloud.points.size()); ++i)
                {
                    const auto& point = cloud.points[i];

                    if ((point.x * point.x + point.y * point.y) > (5.0 * 5.0) ||
                        point.z < 0.1 || point.z > 1.5)
                    {
                        continue;
                    }

                    Eigen::Vector4d pt_camera(point.x, point.y, point.z, 1.0);
                    Eigen::Vector4d pt_map = T.matrix() * pt_camera;

                    double map_x = pt_map(0);
                    double map_y = pt_map(1);

                    int grid_x = (map_x - collision_map.info.origin.position.x) / collision_map.info.resolution;
                    int grid_y = (map_y - collision_map.info.origin.position.y) / collision_map.info.resolution;

                    if (grid_x >= 0 && grid_x < (int)collision_map.info.width &&
                        grid_y >= 0 && grid_y < (int)collision_map.info.height)
                    {
                        int idx = grid_y * collision_map.info.width + grid_x;
                        #pragma omp critical
                        {
                            collision_map.data[idx] = 100;
                        }
                    }
                }
            }

            collision_map.header.stamp = ros::Time::now();
            map_pub_.publish(collision_map);
            ROS_INFO("Published collision map.");

        }
        catch (tf2::TransformException& ex)
        {
            ROS_WARN("TF transform error: %s", ex.what());
        }
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "collision_from_detic_node");
    CollisionFromDetic cfd;
    ros::spin();
    return 0;
}

