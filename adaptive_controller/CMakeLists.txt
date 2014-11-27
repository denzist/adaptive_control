cmake_minimum_required(VERSION 2.8.3)
project(adaptive_controller)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs std_msgs geometry_msgs tf control_msgs message_generation )

add_message_files(
  FILES
  ControlMatrix.msg
  DataSet.msg
  Data.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  control_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(data_assembler src/data_assembler.cpp)

add_dependencies(data_assembler data_assembler_generate_messages_cpp control_msgs_generate_messages_cpp)

target_link_libraries(data_assembler ${catkin_LIBRARIES})
