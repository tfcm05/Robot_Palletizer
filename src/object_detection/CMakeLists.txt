cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  rospy
  sensor_msgs
  cv_bridge
  std_msgs
)

catkin_python_setup()

add_message_files(
  FILES
  DetectionPoint.msg
  DetectionPointArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge std_msgs message_runtime
)

catkin_install_python(PROGRAMS
  scripts/detect_ros.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)