
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
  private_nh.param<std::string>("env_file_name", fn_, "env_furniture_rc24_3330.yaml");
  private_nh.param<bool>("add_static_obstacles", is_adding_, true);

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
  if (is_adding_) {
    ROS_INFO("STARTING DRAWING OBSTACLES");
    drawObstacles();
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
  if(enhanced_map.header.frame_id == "") {
    marker.header.frame_id = "map";
  } else { 
    marker.header.frame_id = enhanced_map.header.frame_id;
  }
  marker.header.stamp = ros::Time();
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  marker.scale.x = (max_x - min_x);
  marker.scale.y = (max_y - min_y);
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

// Convert point to cell coordinates to use in Occupancy Grid Map
int AugmentedGridMap::pointX2Cell(const float x) {
  float x_orig = map_metadata.origin.position.x; 

  float resolution = map_metadata.resolution;

  int cell_x = (x - x_orig )/resolution;
  
  if (cell_x > map_metadata.width )
  {
    ROS_WARN("Point falls x coordinate outside map");
    return map_metadata.width;
  }
  else if (cell_x < 0)
  {
    ROS_WARN("Point falls x coordinate outside map");
    return 0;
  }
  
  return static_cast<int>(cell_x);
}

int AugmentedGridMap::pointY2Cell(const float y) {
  float y_orig = map_metadata.origin.position.y; 

  float resolution = map_metadata.resolution;

  int cell_y = (y - y_orig )/resolution;

  if (cell_y > map_metadata.height )
  {
    ROS_WARN("Point falls x coordinate outside map");
    return map_metadata.width;
  }
  else if (cell_y < 0)
  {
    ROS_WARN("Point falls x coordinate outside map");
    return 0;
  }

  return static_cast<int>(cell_y);
}

void AugmentedGridMap::drawObstacles() {
  // Starting read yaml file //
  // Get path
  std::string pkg_path = ros::package::getPath("hma_env_manage");
  std::string yaml_file = pkg_path + "/io/config/" + fn_;
  
  try //------------------------ Try to load file -----------------------// 
  {
    YAML::Node config = YAML::LoadFile(yaml_file);

    //-------------------- Iterate YAML file ------------------------//
    if (config["furniture"]) {
      for (YAML::const_iterator it = config["furniture"].begin(); it != config["furniture"].end(); ++it) {
        YAML::Node plane_data = it->second["plane"]; // Get points in plane
        
        // Declare list of x&y of points
        std::vector<float> x;
        std::vector<float> y;

        // Extract x, y in each point
        for (const auto& data: plane_data){
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

        // Draw obstacles on Occupancy Grid Map
        for (int i = pointX2Cell(min_x); i < pointX2Cell(max_x); ++i) {
          for (int j = pointY2Cell(min_y); j < pointY2Cell(max_y); ++j) {
            enhanced_map.data[i+j*map_metadata.width] = 100;
          }
        }
        ROS_INFO("Draw on Occupancy Grid Map works!");

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
      } else {
        // Publish enhanced map
        augmented_map_pub.publish( enhanced_map );
        augmented_metadata_pub.publish( map_metadata );

        // Publish obstacle markers
        for (const auto& marker: obstacle_markers_) {
          obstacle_marker_pub.publish(marker);
          ROS_INFO("Publishing markers");
        } 
      }
    }
  } //------------------------ End try -------------------------//
  catch (const YAML::BadFile& e) 
  {
    ROS_ERROR("Failed to load YAML file: %s", yaml_file.c_str());
  }
}

} // end namespace ros_augmented_gridmaps
