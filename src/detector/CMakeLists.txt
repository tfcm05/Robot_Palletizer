cmake_minimum_required(VERSION 3.0.2)
project(detector)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  message_generation
  std_msgs
  sensor_msgs
)

add_message_files(
  FILES
  BoundingBox.msg
  BoundingBoxes.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS
    cv_bridge
    rospy
    message_runtime
    std_msgs
    sensor_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  scripts/detector_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)