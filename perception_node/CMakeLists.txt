cmake_minimum_required(VERSION 2.8.3)
project(perception_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  tf
)

catkin_package()


include_directories(include ./include/perception_node ${catkin_INCLUDE_DIRS} )

#定为节点
add_executable(localization_node src/localization_node.cpp)
target_link_libraries(localization_node ${catkin_LIBRARIES} )
#障碍物感知节点
#add_executable(obstacle_perception_node src/obstacle_perception_node.cpp)
#target_link_libraries(obstacle_perception_node ${catkin_LIBRARIES})

