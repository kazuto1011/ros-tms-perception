cmake_minimum_required(VERSION 2.8.3)
project(tms_ss_cnn)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES tms_ss_cnn
#  CATKIN_DEPENDS cv_bridge rospy sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
