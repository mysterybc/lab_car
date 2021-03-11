cmake_minimum_required(VERSION 2.8.3)
project(ndt_localization)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  pcl_conversions 
  pcl_ros 
  pcl_msgs 
  sensor_msgs
  nav_msgs
  ndt_omp
  tf
  octomap_ros
  octomap_msgs
)

catkin_package()

find_package(OpenMP)

if (OPENMP_FOUND)

set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")

endif()
#需要添加的pcl代码
find_package(OpenCV REQUIRED)

#需要添加的GTSAM代码
#find_package(GTSAM REQUIRED)


#set(GTSAM_LIBRARIES gtsam)   # TODO: automatic search libs

#find_package(GTSAMCMakeTools)
#include(GtsamMakeConfigFile)
#include(GtsamBuildTypes)
#include(GtsamTesting)



include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  #${GTSAM_INCLUDE_DIR}
)

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/add_pcl_link.cpp
# )
#add_library(ndt_omp src/pclomp/voxel_grid_covariance_omp.cpp src/pclomp/ndt_omp.cpp src/pclomp/gicp_omp.cpp)
#add_library(point_cloud_to_octomap_and_OGM  src/point_cloud_to_octomap_and_OGM.cpp)

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
#add_executable(${PROJECT_NAME}_node src/add_pcl_link.cpp)
add_executable(scan_to_submap_node src/scan_to_submap.cpp)
add_executable(pcl_fliter_node src/pcl_fliter.cpp)
add_executable(Eliminating_point_cloud_distortion_node src/Eliminating_point_cloud_distortion.cpp)
add_executable(map_segmentation_node src/map_segmentation.cpp)
add_executable(scan_to_submap_imu_node src/scan_to_submap_imu.cpp include/ndt_localization/ndt_match.h)
add_executable(imu_transform_node src/imu_transform.cpp)
add_executable(scan_to_submap_lego_node src/scan_to_submap_lego.cpp)
add_executable(point_cloud_to_octomap_and_OGM_node src/point_cloud_to_octomap_and_OGM_node.cpp src/point_cloud_to_octomap_and_OGM.cpp)

## Rename C++ executable without prefix
## The above recommended prefix causes long target names, the following renames the
## target back to the shorter version for ease of user use
## e.g. "rosrun someones_pkg node" instead of "rosrun someones_pkg someones_pkg_node"
# set_target_properties(${PROJECT_NAME}_node PROPERTIES OUTPUT_NAME node PREFIX "")

## Add cmake target dependencies of the executable
## same as for the library above

#add_dependencies(${PROJECT_NAME}_node ndt_omp) #这里的align改成你的app

## Specify libraries to link a library or executable target against
#  target_link_libraries(${PROJECT_NAME}_node
#    ${catkin_LIBRARIES} 
#    ${PCL_LIBRARY_DIRS}
#    ndt_omp
#  )



target_link_libraries(scan_to_submap_node
${catkin_LIBRARIES} 
)

target_link_libraries(pcl_fliter_node
 ${catkin_LIBRARIES} 
)

target_link_libraries(Eliminating_point_cloud_distortion_node
${catkin_LIBRARIES} 
)
target_link_libraries(map_segmentation_node
${catkin_LIBRARIES} 
)
target_link_libraries(scan_to_submap_imu_node
${catkin_LIBRARIES} 
)
target_link_libraries(imu_transform_node
${catkin_LIBRARIES} 
)
target_link_libraries(scan_to_submap_lego_node
${catkin_LIBRARIES} 
)
target_link_libraries(point_cloud_to_octomap_and_OGM_node
        ${catkin_LIBRARIES}
        )
# target_link_libraries(gtsam_test_node
# ${catkin_LIBRARIES} 
# ${PCL_LIBRARY_DIRS}
# ${Boost_LIBRARIES} 
# gtsam
# )
#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_executables.html
# install(TARGETS ${PROJECT_NAME}_node
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark libraries for installation
## See http://docs.ros.org/melodic/api/catkin/html/howto/format1/building_libraries.html
# install(TARGETS ${PROJECT_NAME}
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_add_pcl_link.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)



