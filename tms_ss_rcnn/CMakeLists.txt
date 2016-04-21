cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_rcnn)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
)

add_message_files(
  FILES
  object.msg
)

add_service_files(
  DIRECTORY srv
  FILES obj_detection.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS
  LIBRARIES tms_ss_rcnn
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  DEPENDS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

