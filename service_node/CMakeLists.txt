cmake_minimum_required(VERSION 3.0.2)
project(service_node)
add_compile_options(-std=c++11)

find_package(OpenCV 3.3.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  std_msgs
  robot_msgs
  roscpp
)

catkin_package()


include_directories(include 
  ./include/service_node 
  ${catkin_INCLUDE_DIRS} 
  /opt/ros/kinetic/include/opencv-3.3.1-dev
)

#规划节点
add_executable(global_planning 
  src/global_planner.cpp 
  src/tools/AStar.cpp
)
target_link_libraries(global_planning 
  ${catkin_LIBRARIES} 
  /usr/local/lib/libyaml-cpp.so
  ${OpenCV_LIBS}
  
)
#规划节点测试
add_executable(test_global_planning src/test_global_planning.cpp)
target_link_libraries(test_global_planning ${catkin_LIBRARIES} ${OpenCV_LIBS})

