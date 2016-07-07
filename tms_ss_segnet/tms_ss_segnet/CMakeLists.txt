cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_segnet)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_service_files(
  DIRECTORY srv
  FILES segmentation.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES tms_ss_segnet
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
