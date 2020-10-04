cmake_minimum_required(VERSION 3.0.2)
project(communication_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread ") 
set(Jsoncpp_LIBRARIES "/usr/lib/x86_64-linux-gnu/libjsoncpp.so" )
set(ZeroMQ_LIBRARIES "/usr/local/lib/libzmq.a")
find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  roscpp
  std_msgs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})


add_executable(receive_cmd src/receive_cmd.cpp)
target_link_libraries(receive_cmd ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})

add_executable(send_cmd src/send_cmd.cpp)
target_link_libraries(send_cmd ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES} ${catkin_LIBRARIES})

add_executable(send_robot_state_four_car src/send_robot_state_four_car.cpp)
target_link_libraries(send_robot_state_four_car ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES} ${catkin_LIBRARIES})

add_executable(send_robot_state src/send_robot_state.cpp)
target_link_libraries(send_robot_state ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})

add_executable(test_receive src/test_receive.cpp)
target_link_libraries(test_receive ${catkin_LIBRARIES} ${ZeroMQ_LIBRARIES} ${Jsoncpp_LIBRARIES})


