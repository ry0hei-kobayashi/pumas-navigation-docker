
#include "augment_gridmap_online/AugmentedGridMap.hpp"
#include <yaml-cpp/yaml.h>
#include <ros/package.h>

namespace ros_augmented_gridmaps{

AugmentedGridMap::AugmentedGridMap(ros::NodeHandle &nodeHandle)
  :nodeHandle_(nodeHandle)
{ 
   //Private nodehandle only for parameters
  ros::NodeHandle private_nh("~"); 
  private_nh.param<bool>("debug",debug,false);
  private_nh.param<std::string>("input_map",input_map,"map");

  mapSubscriber = nodeHandle_.subscribe(input_map,1,&AugmentedGridMap::saveMap,this);
  // Latched publisher for data
  augmented_map_pub = nodeHandle_.advertise<nav_msgs::OccupancyGrid>("/grid_map/augmented_map", 1, true);
  augmented_map_pub.publish( enhanced_map );
  
  augmented_metadata_pub = nodeHandle_.advertise<nav_msgs::MapMetaData>("/grid_map/augmented_map_metadata", 1, true);
  augmented_metadata_pub.publish( map_metadata );

  obstacle_marker_pub = nodeHandle_.advertise<visualization_msgs::Marker>("/grid_map/obstacle_markers",1,true);

  //Service to clear augmented map
  clearServer = nodeHandle_.advertiseService("/clear_map",&AugmentedGridMap::clearMapCallback,this);

  //Service to publish current augmented map
  getAugmentedMap = nodeHandle_.advertiseService("/grid_map/get_augmented_map",&AugmentedGridMap::getAugmentedMapCallback,this);


  //
  //add by ryohei.k code by nyan
  //
  //////////////////////////////
  // Draw obstacles from file //
  //////////////////////////////

  // Start by reading yaml
  // Get path
  std::string pkg_path = ros::package::getPath("augment_gridmap_online");
  std::string yaml_file = pkg_path + "/io/areas.yaml";
  
  // Load file
  //YAML::Node config = YAML::LoadFile(yaml_file);
  try 
  {
      YAML::Node config = YAML::LoadFile(yaml_file);

      //-------------------- Iterate through each area in YAML file ------------------------//
      for (YAML::const_iterator it = config.begin(); it != config.end(); ++it) {
        // std::string area_name = it->first.as<std::string>(); // Get area_name if needed
        YAML::Node area_data = it->second; // Get points in area
        
        // Declare list of x&y of points
        std::vector<float> x;
        std::vector<float> y;

        // Extract x, y in each point
        for (const auto& data: area_data){
          x.push_back(data[0].as<float>());
          y.push_back(data[1].as<float>());
        }

        // Check if the lists are not empty
        if (x.empty() || y.empty()) {
          ROS_ERROR("Empty list detected. Exiting.");
          return;
        }

        // Initialize min max values
        float min_x = x[0];
        float max_x = x[0];
        float min_y = y[0];
        float max_y = y[0];

        // Find min max of x
        for(const auto& val: x) {
          if (val < min_x) min_x = val;
          if (val > max_x) max_x = val;
        }
        // Find min max of y
        for(const auto& val: y) {
          if (val < min_y) min_y = val;
          if (val > max_y) max_y = val;
        }
        ROS_INFO("min_x: %f, max_x: %f, min_y: %f, max_y: %f", min_x, max_x, min_y, max_y);
        
        // Draw obstacle on Occupancy Grid Map
        for (int i = static_cast<int>(min_x); i < static_cast<int>(max_x) ; i++) {
          for (int j = static_cast<int>(min_y); j < static_cast<int>(max_y) ; j++) {
            enhanced_map.data[i+j*map_metadata.width] = 100;
          }
        }
        
        // Calculate the center point
        float cen_x = 0;
        float cen_y = 0;

        for (const auto& x_coordinate: x) {
          cen_x += x_coordinate;
        }  
        cen_x = cen_x / x.size();

        for (const auto& y_coordinate: y) {
          cen_y += y_coordinate;
        } 
        cen_y = cen_y / y.size();

        ROS_INFO("center point x: %f, y: %f", cen_x, cen_y);
        
        // Prepare marker
        makeObstaclesMarkers(min_x, max_x, min_y, max_y, cen_x, cen_y);
      } //-------------------------- End iterate through yaml file------------------------//


      // Checks if original map has data
      if (original_map.data.size() < 1)
      {
        ROS_ERROR("Do not have original map to enhance yet");
        return;
      }

      // Publish enhanced map
      augmented_map_pub.publish( enhanced_map );

      // Publish obstacle markers
      for (const auto& marker: obstacle_markers_) {
        obstacle_marker_pub.publish(marker);
        ROS_INFO("Publishing markers");
      } 
  } 
  catch (const YAML::BadFile& e) 
  {
      ROS_ERROR("Failed to load YAML file: %s", yaml_file.c_str());
  }

  /////////////////
  // End drawing //
  /////////////////

  ROS_INFO("Map enhancer node Initialization finished");
  return;
}

AugmentedGridMap::~AugmentedGridMap()
{
}

void AugmentedGridMap::saveMap(const nav_msgs::OccupancyGrid &map)
{
  original_map.data = map.data;
  original_map.info = map.info;
  original_map.header.frame_id = map.header.frame_id;
  original_map.header.stamp = map.header.stamp;

  enhanced_map.data = map.data;
  enhanced_map.info = map.info;
  enhanced_map.header.frame_id = map.header.frame_id;
  enhanced_map.header.stamp = ros::Time();
 
  map_metadata = map.info; 
  
  ROS_INFO("Got a map of: [%d,%d] @ %f resolution", enhanced_map.info.width, enhanced_map.info.height, enhanced_map.info.resolution);
  augmented_map_pub.publish( enhanced_map );
  augmented_metadata_pub.publish( map_metadata );
  // Publish obstacle markers
  for (const auto& marker: obstacle_markers_) {
    obstacle_marker_pub.publish(marker);
    ROS_INFO("Publishing markers");
  }
  return;
}

bool AugmentedGridMap::clearMapCallback(std_srvs::Empty::Request& request, 
    std_srvs::Empty::Response& response)
{
  // Copy original map over enhanced one and remove obstacles
  enhanced_map = original_map;
  obstacle_markers_.clear();

  // Publish again
  ROS_INFO("Restoring original map");
  augmented_map_pub.publish( enhanced_map );
  augmented_metadata_pub.publish( map_metadata );
  
  // Publish markers with action DELETE to remove them from RViz
  visualization_msgs::Marker marker_delete;
  marker_delete.action = visualization_msgs::Marker::DELETEALL;
  obstacle_marker_pub.publish(marker_delete);
  
  return true;
}

bool AugmentedGridMap::getAugmentedMapCallback(nav_msgs::GetMap::Request& request, 
    nav_msgs::GetMap::Response& response)
{
  // Copy original map over enhanced one and remove obstacles
  augmented_map_pub.publish( enhanced_map );
  augmented_metadata_pub.publish( map_metadata );

  // Publish again
  ROS_INFO("Getting augmented map");
  response.map = enhanced_map;

  return true;
}

void AugmentedGridMap::makeObstaclesMarkers(const float min_x, const float max_x, const float min_y, const float max_y, const float cen_x, const float cen_y)
{
  // Initialize obstacle marker
  visualization_msgs::Marker marker;
  std_msgs::ColorRGBA color;
  color.r=1.0;
  color.g=0;
  color.b=0;
  color.a=1.0;

  marker.ns = "obstacles";
  marker.action = visualization_msgs::Marker::ADD;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.header.frame_id = enhanced_map.header.frame_id;
  marker.header.stamp = ros::Time();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = (max_x - min_x) / 2.0;
  marker.scale.y = (max_y - min_y) / 2.0;
  marker.scale.z = 0.5; // Obstacle height
  marker.id = obstacle_markers_.size();
  marker.color = color;
  marker.pose.position.x = cen_x;
  marker.pose.position.y = cen_y;
  marker.pose.position.z = 0;

  marker.lifetime = ros::Duration(); // The marker will be persistent
  // Add defined marker to list of obstacle markers
  obstacle_markers_.push_back(marker);
}

}
