cmake_minimum_required(VERSION 2.8.3)
project(leuze_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
)

add_message_files(
  FILES
  ExtendedStatusProfileMsg_rsl200.msg
  ExtendedStatusProfileMsg_rsl400.msg
  PhidgetIKInputMsg_rsl200.msg
  PhidgetIKOutputMsg_rsl200.msg
  PhidgetIKInputMsg_rsl400.msg
  PhidgetIKOutputMsg_rsl400.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)
