cmake_minimum_required(VERSION 3.0.2)
project(robot_library)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  roscpp
  tf
)

catkin_package(
  INCLUDE_DIRS my_lib/include
  LIBRARIES my_lib
  CATKIN_DEPENDS roscpp
)

include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

add_subdirectory(my_lib)
