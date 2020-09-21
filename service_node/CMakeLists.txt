project(service_node)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions
  pcl_ros
  nav_msgs
  geometry_msgs
  actionlib
  robot_msgs
  interactive_markers
  costmap_node
  )

find_package(OpenCV 3.3.0 REQUIRED)
find_package(G2O REQUIRED)
find_package(SUITESPARSE REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(ProtoBuf REQUIRED)
find_package(yaml-cpp REQUIRED)

catkin_package()

add_subdirectory(local_planner)
add_subdirectory(separate_goal)
add_subdirectory(global_planner)


