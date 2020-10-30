cmake_minimum_required(VERSION 3.0.2)
project(decision_node)

set(CMAKE_CXX_FLAGS "${CAMKE_CXX_FLAGS} -std=c++11 -pthread")
find_package(catkin REQUIRED COMPONENTS
  actionlib
  geometry_msgs
  nav_msgs
  robot_msgs
  roscpp
  roslib
  tf
)


catkin_package()


include_directories(
    inc
    inc/exported 
    inc/exported/behaviac_generated/types/internal
  ${catkin_INCLUDE_DIRS}
)

aux_source_directory(src/.  SRC_LIST1)
aux_source_directory(inc/exported/behaviac_generated/types/internal  SRC_LIST2)

add_executable(decision_node ${SRC_LIST1} ${SRC_LIST2})
target_link_libraries(decision_node  pthread ${catkin_LIBRARIES}
${PROJECT_SOURCE_DIR}/libs/libbehaviac_gcc_release.a)



