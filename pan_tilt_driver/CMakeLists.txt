cmake_minimum_required(VERSION 3.0.2)
project(pan_tilt_driver)

add_compile_options(-std=c++14)

find_package(catkin REQUIRED COMPONENTS
  serial
  roscpp
  rospy
  std_msgs
  pan_tilt_msgs
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES pan_tilt_driver
  CATKIN_DEPENDS roscpp rospy std_msgs serial pan_tilt_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(PanTiltDriverNode
  src/PanTiltDriverNode.cpp
  src/PanTiltDriver.cpp
  src/QThread.cpp
  src/modbus_rtu_master.cpp
)  
target_link_libraries(PanTiltDriverNode
  ${catkin_LIBRARIES}
)
add_dependencies(PanTiltDriverNode pan_tilt_msgs_generate_messages_cpp)
