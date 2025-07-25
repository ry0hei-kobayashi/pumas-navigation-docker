#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Trigger.h"
#include "sensor_msgs/Range.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/PointCloud2.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/Vector3.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include "nav_msgs/Path.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

//DEBUG
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
//

#define RATE 30

bool  debug  = false;
bool  enable = false;
bool  use_lidar  = true;
bool  use_cloud  = false;
bool  use_pot_fields = false;
bool  collision_risk_lidar = false;
bool  collision_risk_cloud = false;
float current_speed_linear  = 0;
float current_speed_angular = 0;
float minX =  0.3;
float maxX =  0.7;
float minY = -0.3;
float maxY =  0.3;
float minZ =  0.1;
float maxZ =  1.5;
float pot_fields_d0 = 1.0;
float pot_fields_k_rej = 1.0;
int   cloud_downsampling = 9;
int   cloud_threshold    = 100;
int   lidar_downsampling = 1;
int   lidar_threshold    = 15;
int   no_data_cloud_counter  = 0;
int   no_data_lidar_counter  = 0;
std::string base_link_name      = "base_footprint";
std::string point_cloud_topic   = "/point_cloud";
std::string laser_scan_topic    = "/scan";
geometry_msgs::Vector3 rejection_force_lidar;
geometry_msgs::Vector3 rejection_force_cloud;

ros::NodeHandle* nh;
ros::Subscriber sub_cloud;
ros::Subscriber sub_lidar;
tf::TransformListener* tf_listener;
float global_goal_x = 999999;
float global_goal_y = 999999; 


Eigen::Affine3d get_transform_to_basefootprint(std::string link_name)
{
    tf::StampedTransform tf;
    tf_listener->lookupTransform(base_link_name, link_name, ros::Time(0), tf);
    Eigen::Affine3d e;
    tf::transformTFToEigen(tf, e);
    return e;
}

void get_robot_pose(float& robot_x, float& robot_y, float& robot_t)
{
    tf::StampedTransform transform;
    tf_listener->lookupTransform("map", base_link_name, ros::Time(0), transform);
    robot_x = transform.getOrigin().x();
    robot_y = transform.getOrigin().y();
    tf::Quaternion q = transform.getRotation();
    robot_t = atan2((float)q.z(), (float)q.w()) * 2;
}


float get_search_distance()
{
    ros::param::get("~max_x", maxX);

    float robot_x, robot_y, robot_a;
    get_robot_pose(robot_x, robot_y, robot_a);
    float dist_to_goal = sqrt(pow(global_goal_x - robot_x, 2) + pow(global_goal_x - robot_x, 2));
    if(dist_to_goal < maxX)
        return dist_to_goal;
    return maxX;
}

bool check_collision_risk_with_cloud(sensor_msgs::PointCloud2::Ptr msg, double& rejection_force_x, double& rejection_force_y)
{
    ros::param::get("/obs_detector/pot_fields_k_rej", pot_fields_k_rej);
    ros::param::get("~pot_fields_d0", pot_fields_d0);
    ros::param::get("~max_x", maxX);

    if(current_speed_linear <= 0 && !debug) return false;
    
    float optimal_x = get_search_distance();
    int obstacle_count = 0;
    int force_count = 0;
    rejection_force_x = 0;
    rejection_force_y = 0;
    
    //mask for vision
    int w = msg->width;
    int h = msg->height;
    cv::Mat mask = cv::Mat::zeros(h, w, CV_8UC1);

    unsigned char* p = (unsigned char*)(&msg->data[0]);
    Eigen::Affine3d tf = get_transform_to_basefootprint(msg->header.frame_id);
    for(size_t i=0; i < msg->width*msg->height; i+=cloud_downsampling, p += cloud_downsampling*msg->point_step)
    {
        float s = pot_fields_d0 / maxX;
        Eigen::Vector3d v(*((float*)(p)), *((float*)(p+4)), *((float*)(p+8)));
        v = tf * v;
        if(v.x() > minX && v.x() < optimal_x && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ) obstacle_count ++;
        //if(v.z() > minZ && v.norm() < pot_fields_d0)
        if(v.z() > minZ && v.norm() < maxX)
        {
            mask.data[i] = (unsigned char)255;
            
            float force_mag = s * pot_fields_k_rej*sqrt(1/v.norm() - 1/maxX);
            if (!isnan(force_mag)){
                rejection_force_x -= force_mag*v.x()/v.norm();
                rejection_force_y -= force_mag*v.y()/v.norm();
                force_count++;
            }
        }
    }
    rejection_force_x = force_count > 0 ? rejection_force_x/force_count : 0;
    rejection_force_y = force_count > 0 ? rejection_force_y/force_count : 0;
    
    if(debug)
    {
        if (rejection_force_y > 0)
            std::cout << "ObsDetector.cloud->rejection_force_x: " << rejection_force_x << "  rejection_force_y: " << rejection_force_y << std::endl;

        cv::imshow("OBSTACLE DETECTOR BY MARCOSOFT", mask);
        cv::waitKey(30);
    }


    return obstacle_count > cloud_threshold;
}

bool check_collision_risk_with_lidar(sensor_msgs::LaserScan::Ptr msg, double& rejection_force_x, double& rejection_force_y)
{
    ros::param::get("~pot_fields_k_rej", pot_fields_k_rej);
    ros::param::get("~pot_fields_d0", pot_fields_d0);

    if(current_speed_linear <= 0 && !debug) return false;
    
    float optimal_x = get_search_distance();
    int obstacle_count = 0;
    int force_count = 0;
    rejection_force_x = 0;
    rejection_force_y = 0;
    Eigen::Affine3d tf = get_transform_to_basefootprint(msg->header.frame_id);
    for(size_t i=0; i < msg->ranges.size(); i+=lidar_downsampling)
    {
        float angle = msg->angle_min + i*msg->angle_increment;
        Eigen::Vector3d v(msg->ranges[i]*cos(angle), msg->ranges[i]*sin(angle), 0);
        v = tf * v;
        if(v.x() > minX && v.x() < optimal_x && v.y() > minY && v.y() < maxY && v.z() > minZ && v.z() < maxZ) obstacle_count ++;
        if(v.norm() < pot_fields_d0)
        {
            float force_mag = pot_fields_k_rej*sqrt(1/v.norm() - 1/pot_fields_d0);
            rejection_force_x -= force_mag*v.x()/v.norm();
            rejection_force_y -= force_mag*v.y()/v.norm();
            force_count++;
        }
    }
    rejection_force_x = force_count > 0 ? rejection_force_x/force_count : 0;
    rejection_force_y = force_count > 0 ? rejection_force_y/force_count : 0;
    if(debug){
        if (obstacle_count > lidar_threshold)
            std::cout << "ObsDetector.laser->rejection_force_x: " << rejection_force_x << "  rejection_force_y: " << rejection_force_y << std::endl;
    }
    return obstacle_count > lidar_threshold;
}

void callback_lidar(sensor_msgs::LaserScan::Ptr msg)
{
    no_data_lidar_counter = 0;
    collision_risk_lidar = check_collision_risk_with_lidar(msg, rejection_force_lidar.x, rejection_force_lidar.y);
}

void callback_point_cloud(sensor_msgs::PointCloud2::Ptr msg)
{
    no_data_cloud_counter = 0;
    collision_risk_cloud = check_collision_risk_with_cloud(msg, rejection_force_cloud.x, rejection_force_cloud.y);
}

void callback_goal_path(const nav_msgs::Path::ConstPtr& msg)
{
    //MvnPln sending near goal status
    try{
    //if (!msg->poses.empty()){
        global_goal_x = msg->poses[msg->poses.size() - 1].pose.position.x;
        global_goal_y = msg->poses[msg->poses.size() - 1].pose.position.y;
    }
    catch(...){
    //    global_goal_x = 0.0;
    //    global_goal_y = 0.0;
        global_goal_x = std::numeric_limits<float>::max(); //max->min
        global_goal_y = std::numeric_limits<float>::max();	
    }

    //if (!msg->poses.empty()){
    //    global_goal_x = msg->poses[msg->poses.size() - 1].pose.position.x;
    //    global_goal_y = msg->poses[msg->poses.size() - 1].pose.position.y;
    //TODO ObsDetector is not killed this state, but mvnPln's move error can not converging
    //else{
    //    ROS_ERROR("ObsDetector.->Received an empty path. No goal was set. >>> Recovery");
    //    global_goal_x = std::numeric_limits<float>::max(); //max->min
    //    global_goal_y = std::numeric_limits<float>::max();	
    //}
}

void callbackEnable(const std_msgs::Bool::ConstPtr& msg)
{
    if(msg->data)
    {
        std::cout<<"ObsDetector.->Starting detection using: "<<(use_lidar?"lidar ":"")<<(use_cloud?"point_cloud":"")<<std::endl;
        if(use_cloud ) sub_cloud = nh->subscribe(point_cloud_topic, 1, callback_point_cloud );
        if(use_lidar ) sub_lidar = nh->subscribe(laser_scan_topic , 1, callback_lidar);
    }
    else
    {
        std::cout << "ObsDetector.->Stopping obstacle detection..." <<std::endl;
        if(use_cloud)   sub_cloud.shutdown();
        if(use_lidar)   sub_lidar.shutdown();
        collision_risk_lidar = false;
        collision_risk_cloud = false;
    }
    enable = msg->data;
}

void callback_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg)
{
    current_speed_linear  = msg->linear.x;
    current_speed_angular = msg->angular.z;
}

bool callback_obstacle_in_front(std_srvs::Trigger::Request& req, std_srvs::Trigger::Response& resp)
{
    boost::shared_ptr<sensor_msgs::PointCloud2 const> shared_ptr_cloud;
    boost::shared_ptr<sensor_msgs::LaserScan const>   shared_ptr_lidar;
    sensor_msgs::PointCloud2::Ptr ptr_cloud(new sensor_msgs::PointCloud2());
    sensor_msgs::LaserScan::Ptr   ptr_lidar(new sensor_msgs::LaserScan());
    if(use_cloud) shared_ptr_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic,ros::Duration(10.0));
    if(use_lidar) shared_ptr_lidar = ros::topic::waitForMessage<sensor_msgs::LaserScan>  (laser_scan_topic, ros::Duration(10.0));
    if(use_cloud && shared_ptr_cloud) *ptr_cloud = *shared_ptr_cloud;
    if(use_lidar && shared_ptr_lidar) *ptr_lidar = *shared_ptr_lidar;
    double force_x, force_y;
    resp.success = false;
    bool debug_temp = debug;
    debug = true;
    resp.success |= use_cloud && check_collision_risk_with_cloud(ptr_cloud, force_x, force_y);
    resp.success |= use_lidar && check_collision_risk_with_lidar(ptr_lidar, force_x, force_y);
    debug = debug_temp;
    return true; //This is the flag to indicate the service was executed succesfully, it does not indicate the obstacle
}


visualization_msgs::MarkerArray get_force_arrow_markers(geometry_msgs::Vector3& f1, geometry_msgs::Vector3& f2)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_link_name;
    marker.header.stamp = ros::Time();
    marker.ns = "pot_fields";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.07;
    marker.scale.y = 0.1;
    marker.points.push_back(geometry_msgs::Point());
    marker.points.push_back(geometry_msgs::Point());
    marker.points[1].x = (10.0/pot_fields_k_rej)*f1.x;
    marker.points[1].y = (10.0/pot_fields_k_rej)*f1.y;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    markers.markers.push_back(marker);
    marker.id = 1;
    marker.points[1].x = (10.0/pot_fields_k_rej)*f2.x;
    marker.points[1].y = (10.0/pot_fields_k_rej)*f2.y;
    marker.color.g = 0.5;
    markers.markers.push_back(marker);
    marker.id = 2;
    marker.points[1].x = (10.0/pot_fields_k_rej)*(f1.x + f2.x);
    marker.points[1].y = (10.0/pot_fields_k_rej)*(f1.y + f2.y);
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    markers.markers.push_back(marker);
    return markers;
}

visualization_msgs::Marker createBoundingBoxMarker()
{
    ros::param::get("~max_x", maxX);
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_link_name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "bounding_box";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = (maxX + minX) / 2.0;
    marker.pose.position.y = (maxY + minY) / 2.0;
    marker.pose.position.z = (maxZ + minZ) / 2.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = maxX - minX;
    marker.scale.y = maxY - minY;
    marker.scale.z = maxZ - minZ;
    marker.color.a = 0.5;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    return marker;
}

visualization_msgs::MarkerArray createPotFieldMarkers(geometry_msgs::Vector3& f1, geometry_msgs::Vector3& f2)
{
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker marker;
    marker.header.frame_id = base_link_name;
    marker.header.stamp = ros::Time::now();
    marker.ns = "pot_fields";
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.1;  // arrow shaft
    marker.scale.y = 0.2;  // arrow head
    marker.color.a = 1.0; 

    //lidar pot field
    marker.id = 0;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 1.0;
    marker.points.push_back(geometry_msgs::Point());
    geometry_msgs::Point point;
    point.x = f1.x;
    point.y = f1.y;
    point.z = f1.z;
    marker.points.push_back(point);
    markers.markers.push_back(marker);

    //pcl pot field
    marker.id = 1;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.points.clear();
    marker.points.push_back(geometry_msgs::Point());
    point.x = f2.x;
    point.y = f2.y;
    point.z = f2.z;
    marker.points.push_back(point);
    markers.markers.push_back(marker);

    //lidar+pcl pot field
    marker.id = 2;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.points.clear();
    marker.points.push_back(geometry_msgs::Point());
    point.x = f1.x + f2.x;
    point.y = f1.y + f2.y;
    point.z = f1.z + f2.z;
    marker.points.push_back(point);
    markers.markers.push_back(marker);

    return markers;
}


int main(int argc, char** argv)
{
    std::cout << "INITIALIZING OBSTACLE DETECTOR NODE BY MARCO NEGRETE... " << std::endl;
    ros::init(argc, argv, "obs_detect");
    ros::NodeHandle n;
    ros::Rate loop(RATE);
    tf_listener = new tf::TransformListener();
    nh = &n;

    float no_sensor_data_timeout = 0.5;
    ros::param::param<bool >("~debug", debug, false);
    ros::param::param<bool >("~use_pot_fields", use_pot_fields, false);
    ros::param::param<bool >("~use_lidar", use_lidar, true);
    ros::param::param<bool >("~use_point_cloud", use_cloud, false);
    ros::param::param<float>("~min_x", minX,  0.30);
    ros::param::param<float>("~max_x", maxX,  0.80);
    ros::param::param<float>("~min_y", minY, -0.30);
    ros::param::param<float>("~max_y", maxY,  0.30);
    ros::param::param<float>("~min_z", minZ,  0.05);
    ros::param::param<float>("~max_z", maxZ,  1.50);
    ros::param::param<float>("~pot_fields_k_rej", pot_fields_k_rej, 1.0);
    ros::param::param<float>("~pot_fields_d0", pot_fields_d0, 1.0);
    ros::param::param<float>("~no_sensor_data_timeout" , no_sensor_data_timeout, 0.5);
    ros::param::param<int  >("~cloud_points_threshold" , cloud_threshold, 100);
    ros::param::param<int  >("~cloud_downsampling"     , cloud_downsampling, 9);
    ros::param::param<int  >("~lidar_points_threshold" , lidar_threshold, 10);
    ros::param::param<int  >("~lidar_downsampling"     , lidar_downsampling, 1);
    ros::param::param<std::string>("~point_cloud_topic", point_cloud_topic, "/points");
    ros::param::param<std::string>("~laser_scan_topic" ,  laser_scan_topic , "/scan"  );
    ros::param::param<std::string>("/base_link_name"   , base_link_name, "base_link");

    std::cout << "ObsDetector.->Starting obs detection using: "<<(use_lidar?"lidar ":"")<<(use_cloud?"point_cloud ":"")<<std::endl;
    std::cout << "ObsDetector.->Using parameters: min_x=" << minX << "  max_x=" << maxX << "  min_y=" << minY << "  max_y=";
    std::cout << maxY << "  min_z=" << minZ << "  max_z=" << maxZ << std::endl;
    std::cout << "ObsDetector.->Point cloud topic: "<<point_cloud_topic<<"   lidar topic name: " << laser_scan_topic << std::endl;
    std::cout << "ObsDetector.->Params for cloud: threshold="<<cloud_threshold<<"  downsampling="<<cloud_downsampling<<std::endl;
    std::cout << "ObsDetector.->Params for lidar: threshold="<<lidar_threshold<<"  downsampling="<<lidar_downsampling<<std::endl;
    std::cout << "ObsDetector.->Calculate potential fields: " << (use_pot_fields?"True":"False") << std::endl;
    std::cout << "ObsDetector.->Base link frame: " << base_link_name << std::endl;

    std::cout << "ObsDetector.->Waiting for first messages from active sensors: ";
    std::cout << (use_cloud ? point_cloud_topic : "" ) << " " << (use_lidar ? laser_scan_topic : "") << std::endl;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> ptr_cloud_temp; 
    boost::shared_ptr<sensor_msgs::LaserScan const>   ptr_lidar_temp;
    
    for(int i=0; i<10 && use_cloud && ptr_cloud_temp==NULL; i++)
        ptr_cloud_temp = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(point_cloud_topic,ros::Duration(1.0));
    for(int i=0; i<10 && use_lidar && ptr_lidar_temp==NULL; i++)
        ptr_lidar_temp = ros::topic::waitForMessage<sensor_msgs::LaserScan>  (laser_scan_topic, ros::Duration(1.0));
    if(use_cloud && ptr_cloud_temp == NULL)
    {
        std::cout << "ObsDetector.->Cannot get first message for cloud from topic " << point_cloud_topic << std::endl;
        return -1;
    }
    if(use_lidar && ptr_lidar_temp == NULL)
    {
    std::cout << "ObsDetector.->Cannot get first message for lidar from topic " << laser_scan_topic << std::endl;
        return -1;
    }
    std::cout << "ObsDetector.->First messages received..." << std::endl;
    
    std::cout << "ObsDetector.->Waiting for transforms to be available..." << std::endl;
    tf_listener->waitForTransform("map", base_link_name, ros::Time(0), ros::Duration(10.0));
    std::cout << "ObsDetector.->Waiting for sensor transforms" << std::endl;
    if(use_cloud) tf_listener->waitForTransform(base_link_name,ptr_cloud_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    if(use_lidar) tf_listener->waitForTransform(base_link_name,ptr_lidar_temp->header.frame_id,ros::Time(0),ros::Duration(10.0));
    std::cout << "ObsDetector.->Sensor transforms are now available"<< std::endl;

    ros::Subscriber subEnable   = n.subscribe("/navigation/obs_detector/enable", 1, callbackEnable);
    ros::Subscriber sub_cmd_vel = n.subscribe("/cmd_vel", 1, callback_cmd_vel);
    ros::Subscriber sub_goal_path      = n.subscribe("/simple_move/goal_path", 10, callback_goal_path);
    ros::Publisher  pub_collision_risk = n.advertise<std_msgs::Bool>("/navigation/obs_detector/collision_risk", 1);
    ros::Publisher  pub_pot_fields_mrk = n.advertise<visualization_msgs::MarkerArray>("/navigation/obs_detector/pot_field_markers", 1);
    ros::Publisher  pub_pot_fields_rej = n.advertise<geometry_msgs::Vector3>("/navigation/obs_detector/pf_rejection_force", 1);
    std_msgs::Bool msg_collision_risk;
    geometry_msgs::Vector3 msg_rejection_force;

    //for visualize bbox and pot fields
    ros::Publisher pub_bounding_box = n.advertise<visualization_msgs::Marker>("bounding_box_marker", 1);
    ros::Publisher pub_pot_fields = n.advertise<visualization_msgs::MarkerArray>("pot_fields_marker", 1);
        
    while(ros::ok())
    {
        // add by rk 2024/8/25 obstacle detectionをon offする 
        bool current_cloud_status;
        bool current_lidar_status;
        ros::param::get("~use_point_cloud", current_cloud_status);
        ros::param::get("~use_lidar", current_lidar_status);
    
        //lidar
        if (use_lidar != current_lidar_status)
        {
            use_lidar = current_lidar_status;
            if (use_lidar) {
                sub_lidar = nh->subscribe(laser_scan_topic, 1, callback_lidar);
                ROS_WARN("ObsDetector.->Lidar subscription enabled.");
            } else {
                sub_lidar.shutdown();
                ROS_WARN("ObsDetector.->Lidar subscription disabled.");
            }
        }
        //point cloud
        if (use_cloud != current_cloud_status)
        {
            use_cloud = current_cloud_status;
            if (use_cloud) {
                sub_cloud = nh->subscribe(point_cloud_topic, 1, callback_point_cloud);
                std::cout << "ObsDetector.->Point cloud subscription enabled." << std::endl;
            } else {
                sub_cloud.shutdown();
                std::cout << "ObsDetector.->Point cloud subscription disabled." << std::endl;
            }
        }

        //obstacle detectionがenableの時，障害物を回避する, pumas original
        if(enable)
        {
            msg_collision_risk.data = collision_risk_lidar || collision_risk_cloud;
            pub_collision_risk.publish(msg_collision_risk);
            if(use_pot_fields)
            {
                msg_rejection_force.x = rejection_force_lidar.x + rejection_force_cloud.x;
                msg_rejection_force.y = rejection_force_lidar.y + rejection_force_cloud.y;

                if (rejection_force_lidar.y > 0 && rejection_force_cloud.y > 0){
                    msg_rejection_force.x = (rejection_force_lidar.x + rejection_force_cloud.x)/2;
                    msg_rejection_force.y = (rejection_force_lidar.y + rejection_force_cloud.y)/2;
                }

                pub_pot_fields_rej.publish(msg_rejection_force);
                pub_pot_fields_mrk.publish(get_force_arrow_markers(rejection_force_lidar, rejection_force_cloud));
            
                //for visualize bbox
                visualization_msgs::Marker bounding_box_marker = createBoundingBoxMarker();
                pub_bounding_box.publish(bounding_box_marker);
                //for visualize pot fields
                visualization_msgs::MarkerArray pot_field_markers = createPotFieldMarkers(rejection_force_lidar, rejection_force_cloud);
                pub_pot_fields.publish(pot_field_markers);
            }

            //if(use_lidar  && no_data_lidar_counter++ > no_sensor_data_timeout*RATE)
            //    std::cout << "ObsDetector.->WARNING!!! No lidar data received from topic: " << laser_scan_topic << std::endl;
            //if(use_cloud  && no_data_cloud_counter++ > no_sensor_data_timeout*RATE)
            //    std::cout << "ObsDetector.->WARNING!!! No cloud data received from topic: " << point_cloud_topic << std::endl;              
        }

        ros::spinOnce();
        loop.sleep();
    }
    delete tf_listener;
}
