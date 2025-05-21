cmake_minimum_required(VERSION 3.0.2)
project(pan_tilt_description)

find_package(catkin REQUIRED COMPONENTS
  rospy
  urdf
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/simple_ptz_controller.py
  scripts/guardar_referencia.py
  scripts/foto.py
  scripts/flow.py
  scripts/auto.py
  scripts/real.py
  scripts/auto_funcional.py
  scripts/resaltar_punto.py
  scripts/iou.py
  scripts/optical.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
