cmake_minimum_required(VERSION 3.0.2)
project(robot_sim_test)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  gazebo_plugins
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf
  urdf
  xacro
)

find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES wpr_plugin_rst
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs sensor_msgs nav_msgs tf gazebo_ros gazebo_plugins
  DEPENDS gazebo
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

# WPR base controller plugin consumed by wpb_home.model
# Target name suffixed to avoid collision with wpr_simulation's wpr_plugin target;
# OUTPUT_NAME keeps the produced .so as libwpr_plugin.so (as referenced by the model).
add_library(wpr_plugin_rst SHARED src/wpr_plugin.cpp)
set_target_properties(wpr_plugin_rst PROPERTIES OUTPUT_NAME wpr_plugin)
target_link_libraries(wpr_plugin_rst ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

install(DIRECTORY launch models worlds meshes config scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
install(TARGETS wpr_plugin_rst
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(
  PROGRAMS
    scripts/mock_grasp_executor.py
    scripts/trigger_schedule.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scripts
)
