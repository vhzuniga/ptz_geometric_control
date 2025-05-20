cmake_minimum_required(VERSION 3.0.2)
project(pan_tilt_bringup)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  roscpp
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
