cmake_minimum_required(VERSION 2.8.3)
project(simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  geometry_msgs
  roscpp
  robot_msgs
  std_msgs
  tf
)

# find_package(Boost REQUIRED COMPONENTS system)

catkin_package(

)

include_directories(
# include
../../robot_library/my_lib/include
  ${catkin_INCLUDE_DIRS}
)



add_executable(host_cmd_pub src/host_cmd_pub.cpp)
target_link_libraries(host_cmd_pub ${catkin_LIBRARIES})

add_executable(send_march_mission src/send_march_mission.cpp)
target_link_libraries(send_march_mission ${catkin_LIBRARIES})

add_executable(test_pathCoverage src/test_pathCoverage.cpp)
target_link_libraries(test_pathCoverage ${catkin_LIBRARIES})


add_executable(test_search src/test_search.cpp)
target_link_libraries(test_search ${catkin_LIBRARIES})

add_executable(test_gridmap src/test.cpp)
target_link_libraries(test_gridmap ${catkin_LIBRARIES})
