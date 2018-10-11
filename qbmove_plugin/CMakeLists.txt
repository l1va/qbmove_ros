cmake_minimum_required(VERSION 2.8.3)
project(qbmove_plugin)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  message_generation
)

find_package(gazebo REQUIRED)

add_message_files(
  FILES
#  Position.msg
#  Command.msg
)

generate_messages()

link_directories(${GAZEBO_LIBRARY_DIRS} ${catkin_LIBRARY_DIRS})
include_directories(include ${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
	qbmove_plugin
  DEPENDS 
    roscpp
    gazebo_ros
    message_runtime
    std_msgs 
)

add_library(qbmove_plugin src/qbmove_plugin.cpp)
target_link_libraries(qbmove_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})


