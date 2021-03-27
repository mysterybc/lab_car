cmake_minimum_required(VERSION 3.0.2)
project(decision_node)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
find_package(catkin REQUIRED COMPONENTS
    tf
  nav_msgs
  geometry_msgs
  actionlib
  robot_msgs
  roscpp
  roslib
  robot_library
  apriltag_ros
)

catkin_package(
)

include_directories(
    inc
    inc/exported 
    inc/exported/behaviac_generated/types/internal
    ../robot_library/my_lib/include
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/.  SRC_LIST1)
aux_source_directory(inc/exported/behaviac_generated/types/internal  SRC_LIST2)

 add_executable(decision_node ${SRC_LIST1} ${SRC_LIST2})
 target_link_libraries(decision_node  pthread ${catkin_LIBRARIES}
 ${CMAKE_CURRENT_SOURCE_DIR}/libs/libbehaviac_gcc_release.a)

