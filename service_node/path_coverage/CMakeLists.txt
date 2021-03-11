## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  move_base_msgs
  roscpp
)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})


find_package(catkin REQUIRED
  COMPONENTS
    angles
    costmap_2d
    geometry_msgs
    nav_msgs
    roscpp
    tf
)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(path_coverage src/PathPlanningNode.cpp src/CleaningPathPlanner.cpp src/LocalGridVelocityComputer.cpp src/room_rotator.cpp)
target_link_libraries(path_coverage ${catkin_LIBRARIES} ${OpenCV_LIBS})


