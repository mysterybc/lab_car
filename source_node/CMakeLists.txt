cmake_minimum_required(VERSION 2.8.3)
project(source_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  robot_msgs
  tf
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS})

#add_dependencies(encoder_source_node source_node_generate_messages_cpp)
#编码器节点
add_executable(encoder_source_node src/encoder_source.cpp src/komodo/encoder.cpp)
target_link_libraries(encoder_source_node ${catkin_LIBRARIES})
#组合导航节点
add_executable(navigation_source_node src/navigation_source.cpp)
target_link_libraries(navigation_source_node ${catkin_LIBRARIES})
#驱动器去节点
add_executable(driver_source_node src/driver_source.cpp)
target_link_libraries(driver_source_node ${catkin_LIBRARIES})
#摄像头节点待添加


