cmake_minimum_required(VERSION 3.0.2)
project(task_planning)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  geometry_msgs
  tf
  object_detection
)

catkin_package(
  CATKIN_DEPENDS rospy std_msgs geometry_msgs tf object_detection
)

catkin_install_python(PROGRAMS
  scripts/task_planning_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
