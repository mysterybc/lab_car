add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  message_generation
  actionlib
  roscpp
  tf
  roslib
)

# Generate services in the 'srv' folder
# add_service_files(
#   DIRECTORY srv
#   FILES
#   AddNum.srv
# )

# # # Generate actions in the 'action' folder
# add_action_files(
#   DIRECTORY action
#   FILES
#   WashDish.action
# )

# # # Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs  # Or other packages containing msgs
#   actionlib_msgs
# )

# catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES my_lib
#   CATKIN_DEPENDS roscpp
# )



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(my_lib 
            src/lib/my_param_server.cpp)

add_executable(change_ros_version src/change_ros_version.cpp)
target_link_libraries(change_ros_version ${catkin_LIBRARIES})

# add_executable(my_action_demo src/demo/my_action_demo.cpp )
# target_link_libraries(my_action_demo ${catkin_LIBRARIES})

# add_executable(my_service_demo src/demo/my_service_demo.cpp )
# target_link_libraries(my_service_demo ${catkin_LIBRARIES})

# add_executable(my_param_server_demo src/demo/my_param_server_demo.cpp )
# target_link_libraries(my_param_server_demo ${catkin_LIBRARIES} my_lib)

