cmake_minimum_required(VERSION 3.0.2)
project(task_planning)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  robot_core
  object_detection
  message_generation
)

add_message_files(
  FILES
  TaskCommand.msg
  TaskResult.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    std_msgs
    geometry_msgs
    robot_core
    object_detection
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(task_planning_node
  src/task_planning_node.cpp
)

add_dependencies(task_planning_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

add_compile_options(-std=c++17)

target_link_libraries(task_planning_node
  ${catkin_LIBRARIES}
)

install(TARGETS task_planning_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
