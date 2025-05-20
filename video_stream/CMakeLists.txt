cmake_minimum_required(VERSION 3.0.2)
project(video_stream)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  std_srvs
  message_generation
)

add_service_files(
  FILES
  SetZoomLevel.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
)

catkin_package(
  CATKIN_DEPENDS message_runtime rospy std_msgs std_srvs sensor_msgs cv_bridge
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/video_stream_node.py
  scripts/zoom_calibrator.py
  scripts/zoom_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
