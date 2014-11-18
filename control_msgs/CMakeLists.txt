cmake_minimum_required(VERSION 2.8.3)
project(control_msgs)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs geometry_msgs tf message_generation)

add_message_files(
  FILES
  ControlVector.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)
