cmake_minimum_required(VERSION 3.0.2)
project(communication_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread ") 
set(Jsoncpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libjsoncpp.so" )
set(ZeroMQ_LIBRARIES "/usr/local/lib/libzmq.a")
find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  roscpp
  std_msgs
  tf
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(zmq_test src/zmq_test.cpp)


add_executable(receive_host_cmd src/receive_host_cmd.cpp  )
target_link_libraries(receive_host_cmd zmq_test ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})

add_executable(publish_robot_state src/publish_robot_state.cpp )
target_link_libraries(publish_robot_state zmq_test ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})

add_executable(receive_robot_state src/receive_robot_state.cpp  )
target_link_libraries(receive_robot_state zmq_test ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})



