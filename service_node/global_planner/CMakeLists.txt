cmake_minimum_required(VERSION 3.0.2)
project(global_planner)


include_directories(include 
  ${catkin_INCLUDE_DIRS} 
)

#规划节点
add_executable(global_planning 
  src/global_planner.cpp 
  src/tools/AStar.cpp
)
target_link_libraries(global_planning 
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBS}
  yaml-cpp
)


#规划节点测试
add_executable(test_global_planning src/test_global_planning.cpp)
target_link_libraries(test_global_planning ${catkin_LIBRARIES} ${OpenCV_LIBS})

