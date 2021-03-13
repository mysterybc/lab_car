cmake_minimum_required(VERSION 3.0.2)
project(task_node)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  robot_msgs
  roscpp
  std_msgs
  roslib
  tf
  robot_library
)

set(bitctrl_DIR ../robot_library/bitrobot/cpp/install-all/lib/cmake/bitctrl)
find_package(bitctrl REQUIRED)

catkin_package(
)

include_directories(include ${catkin_INCLUDE_DIRS} ../robot_library/my_lib/include)


####单车的####
add_executable(build_up_task src/build_up_task.cpp)
target_link_libraries(build_up_task ${catkin_LIBRARIES})

####没有host####

add_executable(build_up_task_nohost src/build_up_task_nohost.cpp)
target_link_libraries(build_up_task_nohost ${catkin_LIBRARIES} )


####march action####

add_executable(gps_march_task src/gps_march_task.cpp)
target_link_libraries(gps_march_task ${catkin_LIBRARIES} bitctrl )


add_executable(laser_march_task src/laser_march_task.cpp)
target_link_libraries(laser_march_task ${catkin_LIBRARIES} bitctrl )

####search action####

add_executable(search_task src/search_task.cpp)
target_link_libraries(search_task ${catkin_LIBRARIES} )


