cmake_minimum_required(VERSION 3.0.2)
project(communication_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread ") 

find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(zmq_recv src/zmq_recv.cpp)
target_link_libraries(zmq_recv /usr/local/lib/libzmq.a ${catkin_LIBRARIES})

add_executable(zmq_send src/zmq_send.cpp)
target_link_libraries(zmq_send /usr/local/lib/libzmq.a ${catkin_LIBRARIES})


