cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS robot_library actionlib roslib)

set(bitctrl_DIR ../../robot_library/bitrobot/cpp/install-all/lib/cmake/bitctrl)
find_package(bitctrl REQUIRED)


include_directories(include ${catkin_INCLUDE_DIRS} ../../robot_library/my_lib/include)

add_executable(path_follow src/path_follow.cpp)
target_link_libraries(path_follow ${catkin_LIBRARIES} bitctrl)