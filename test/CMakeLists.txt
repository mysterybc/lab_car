cmake_minimum_required(VERSION 3.0.2)
project(test)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  sensor_msgs
  tf
)

# find_package(Boost REQUIRED COMPONENTS system)
catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

#测试速度指令和反馈
add_executable(test_twist src/test_twist.cpp)
target_link_libraries(test_twist
   ${catkin_LIBRARIES}
)


