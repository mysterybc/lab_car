cmake_minimum_required(VERSION 2.8.3)
project(simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  robot_msgs
  std_msgs
  tf
  turtlebot3_navigation
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


add_executable(tf_pub src/tf_pub.cpp)
target_link_libraries(tf_pub ${catkin_LIBRARIES})

add_executable(host_cmd_pub src/host_cmd_pub.cpp)
target_link_libraries(host_cmd_pub ${catkin_LIBRARIES})
