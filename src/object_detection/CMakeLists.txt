cmake_minimum_required(VERSION 3.0.2)
project(object_detection)

find_package(catkin REQUIRED COMPONENTS
	cv_bridge
	tf
	roscpp
	rospy
	std_msgs
	sensor_msgs
  	geometry_msgs
  	message_generation
	message_filters
  	pcl_ros
  	pcl_conversions
)

find_package(OpenCV REQUIRED)

find_package(PCL REQUIRED)

add_message_files(
	FILES
	DetectionObject.msg
	DetectionObjects.msg
)

generate_messages(
	DEPENDENCIES
	std_msgs
	sensor_msgs
	geometry_msgs
)

catkin_package(
	CATKIN_DEPENDS
	cv_bridge
	tf
	roscpp
	rospy
	std_msgs
    sensor_msgs
    geometry_msgs
	message_runtime
	message_filters
    pcl_ros
    pcl_conversions
)

include_directories(
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(object_detection_node
	src/object_detection_node.cpp
)

add_dependencies(object_detection_node
	${${PROJECT_NAME}_EXPORTED_TARGETS}
	${catkin_EXPORTED_TARGETS}
)

add_compile_options(-std=c++17)

target_link_libraries(object_detection_node
	${catkin_LIBRARIES}
	${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

install(TARGETS object_detection_node
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch config
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
