cmake_minimum_required(VERSION 3.0.2)
project(obstacle_detector)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf_conversions
  visualization_msgs
  detic_ros
)


find_package(OpenCV REQUIRED)
set(OpenCV_INCLUDE_DIRS "/usr/local/include")
find_package(PCL REQUIRED)

find_package(OpenMP REQUIRED)

catkin_package(
  CATKIN_DEPENDS
  detic_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)
add_executable(obs_detect_node src/obs_detect_node.cpp)
add_executable(collision_from_detic_node src/collision_from_detic.cpp)


add_dependencies(obs_detect_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
add_dependencies(collision_from_detic_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(obs_detect_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

target_link_libraries(collision_from_detic_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

if(OpenMP_CXX_FOUND)
    target_link_libraries(collision_from_detic_node OpenMP::OpenMP_CXX)
endif()
