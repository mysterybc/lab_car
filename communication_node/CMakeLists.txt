cmake_minimum_required(VERSION 3.0.2)
project(communication_node)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread ") 

#find libzmq and jsoncpp
find_package(PkgConfig REQUIRED)
pkg_check_modules(ZMQ libzmq)
pkg_check_modules(JSONCPP jsoncpp)
message(${ZMQ_LIBRARIES})

find_package(catkin REQUIRED COMPONENTS
  robot_msgs
  roscpp
  std_msgs
  tf
  robot_library
)

catkin_package()

#library path ??why need??
link_directories(${ZMQ_LIBRARY_DIRS})


set(libs ${catkin_LIBRARIES} ${ZMQ_LIBRARIES} ${JSONCPP_LIBRARIES} )
set(incs include ${catkin_INCLUDE_DIRS} ${ZMQ_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ../robot_library/my_lib/include)

#include
include_directories(${incs})
add_executable(receiver src/receiver.cpp  )
target_link_libraries(receiver  ${libs} )

add_executable(publisher src/publisher.cpp )
target_link_libraries(publisher  ${libs})

add_executable(test_recv src/test_recv.cpp )
target_link_libraries(test_recv  ${libs})

add_executable(transport_image src/transport_image.cpp )
target_link_libraries(transport_image  ${libs})

#add_executable(receive_robot_state src/receive_robot_state.cpp  )
#target_link_libraries(receive_robot_state zmq_test ${catkin_LIBRARIES} zmq)



