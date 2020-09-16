project(costmap_node)
cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  pcl_conversions
  pcl_ros
  laser_geometry
  sensor_msgs
  geometry_msgs
  roslib
  )

find_package(PCL 1.7 REQUIRED)
find_package(Boost REQUIRED)
find_package(roslib)

catkin_package(
  INCLUDE_DIRS
  include
)

include_directories(include/costmap_node /usr/include/pcl-1.7/pcl /usr/local/include/yaml-cpp)
aux_source_directory(src/. SRC_LIST)
list(REMOVE_ITEM SRC_LIST "src/test_costmap.cpp")

#lib project
add_library(costmap_node
  SHARED
  ${SRC_LIST}
  )

target_include_directories(costmap_node
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

target_link_libraries(costmap_node
  PUBLIC
  /usr/local/lib/libyaml-cpp.so
  ${catkin_LIBRARIES}
)

#test project
add_executable(test_costmap src/test_costmap.cpp)

target_include_directories(test_costmap
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  )

target_link_libraries(test_costmap
  costmap_node
  ${catkin_LIBRARIES}
  /usr/local/lib/libyaml-cpp.so
  )

list(APPEND catkin_LIBRARIES costmap_node)

install(DIRECTORY include
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
  )


