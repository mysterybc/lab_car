cmake_minimum_required(VERSION 2.8.3)
project(source_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  robot_msgs
  robot_library
  tf
)

catkin_package()


include_directories(include ${catkin_INCLUDE_DIRS} ../robot_library/my_lib/include)

#组合导航节点
add_executable(navigation_source_node 
  src/navigation_source.cpp 
  src/komodo/imu.cpp 
  src/komodo/gps.cpp
)
target_link_libraries(navigation_source_node ${catkin_LIBRARIES})

#驱动器节点(编码器读取也在里面)
add_executable(driver_source_node 
  src/driver_source.cpp 
  src/komodo/encoder.cpp
)
target_link_libraries(driver_source_node ${catkin_LIBRARIES})
#摄像头节点待添加


#传感器融合节点
add_executable(sensor_fusion_node  src/sensor_fusion.cpp)
target_link_libraries(sensor_fusion_node ${catkin_LIBRARIES})


