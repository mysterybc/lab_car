cmake_minimum_required(VERSION 3.0.2)
project(separate_goal)

find_package(catkin REQUIRED COMPONENTS robot_library tf)
include_directories(include 
  ${catkin_INCLUDE_DIRS} 
  ../../robot_library/my_lib/include
)

#散点节点
add_executable(separate_goal 
  src/separate_goal.cpp 
)
target_link_libraries(separate_goal 
  ${catkin_LIBRARIES} 
)