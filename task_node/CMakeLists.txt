cmake_minimum_required(VERSION 3.0.2)
project(task_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11") 

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

#car1
add_executable(build_up_task1 src/build_up_task1.cpp)
target_link_libraries(build_up_task1 ${catkin_LIBRARIES})
#car2
add_executable(build_up_task2 src/build_up_task2.cpp)
target_link_libraries(build_up_task2 ${catkin_LIBRARIES})
#car3
add_executable(build_up_task3 src/build_up_task3.cpp)
target_link_libraries(build_up_task3 ${catkin_LIBRARIES})
#car4
add_executable(build_up_task4 src/build_up_task4.cpp)
target_link_libraries(build_up_task4 ${catkin_LIBRARIES})

