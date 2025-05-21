cmake_minimum_required(VERSION 3.0.2)
project(pan_tilt_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  rospy
  std_msgs
)

## Registrar archivos .msg
add_message_files(
  FILES
  PanTiltCmdDeg.msg
  PanTiltCmdRad.msg
  PanTiltStatus.msg
)

## Generar mensajes con dependencias
generate_messages(
  DEPENDENCIES std_msgs
)

## Configuración del paquete catkin
catkin_package(
  CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
)

## Directorios de inclusión (no usado aquí, pero necesario)
include_directories(
  ${catkin_INCLUDE_DIRS}
)
