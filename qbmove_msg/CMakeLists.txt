cmake_minimum_required(VERSION 2.8.3)
project(qbmove_msg)

find_package(catkin REQUIRED COMPONENTS rospy message_generation)

add_message_files(FILES Command.msg Position.msg)

generate_messages()

catkin_package()

include_directories(${catkin_INCLUDE_DIRS})
