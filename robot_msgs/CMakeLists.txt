cmake_minimum_required(VERSION 2.8.3)
project(robot_msgs)
set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR})

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  actionlib
  tf
)

add_message_files(
  DIRECTORY msg
  FILES
  Mission.msg
  HostCmd.msg
  Error.msg
  State.msg
  Cmd.msg
  PerceptionNodeMsg.msg
  SourceNodeMsg.msg
  RobotState.msg
  RobotStates.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  Separate.srv
  ReportPath.srv
  GetConfigCmd.srv
)

add_action_files(
    DIRECTORY action
    FILES
    BuildUp.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
)

