cmake_minimum_required(VERSION 3.0.2)
project(task_node)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robot_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


####单车的####
add_executable(build_up_task src/build_up_task.cpp)
target_link_libraries(build_up_task ${catkin_LIBRARIES})

####四车的####

add_executable(build_up_task_multi_car src/build_up_task_multi_car.cpp)
target_link_libraries(build_up_task_multi_car ${catkin_LIBRARIES})


