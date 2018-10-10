cmake_minimum_required(VERSION 2.8.3)
project(qb_real)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
 add_message_files(
   FILES
#   Position.msg
#   Command.msg
 )
generate_messages()
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES qb_real
#  CATKIN_DEPENDS rospy
  DEPENDS
  message_runtime
  std_msgs
)
