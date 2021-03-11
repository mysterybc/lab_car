project(service_node)
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
  roslib
  )

include_directories(include ${catkin_INCLUDE_DIRS} ../../robot_library/my_lib/include)

catkin_package()


add_subdirectory(separate_goal)
add_subdirectory(path_coverage)
add_subdirectory(path_follow)


