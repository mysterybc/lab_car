project(decision_node)
cmake_minimum_required(VERSION 3.1)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_module)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  geometry_msgs
  actionlib
  robot_msgs
  )

find_package(Eigen3 REQUIRED)

include_directories(
  include
  ../decision_node
  ${catkin_INCLUDE_DIRS}
)


catkin_package()

add_executable(behavior_node
  example_behavior/build_up_behavior.h
  behavior_tree.cpp
  )

target_link_libraries(behavior_node
  PRIVATE
  ${catkin_LIBRARIES}
  )
