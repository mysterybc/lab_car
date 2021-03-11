cmake_minimum_required(VERSION 2.8.3)
project(control_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  robot_msgs
  robot_library
)


catkin_package()


include_directories(include  ${catkin_INCLUDE_DIRS} ../robot_library/my_lib/include)

#遥控节点
add_executable(remote_control_node src/remote_control_node.cpp)
target_link_libraries(remote_control_node ${catkin_LIBRARIES})
#单车轨线跟踪
add_library(AlgoTracking src/Tracking/AlgoTracking.cpp)
add_library(Tkspline src/Tracking/tkspline.cpp)
add_executable(path_tracking_node src/path_tracking_node.cpp  )
target_link_libraries(path_tracking_node ${catkin_LIBRARIES} AlgoTracking Tkspline) 

