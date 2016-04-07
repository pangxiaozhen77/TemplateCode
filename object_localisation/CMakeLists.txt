cmake_minimum_required(VERSION 2.8.3)
project(object_localisation)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  std_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS 
  	include
  LIBRARIES 
  	object_localisation
  CATKIN_DEPENDS 
  	pcl_conversions 
  	pcl_ros roscpp 
  	rospy sensor_msgs 
  	std_msgs 
  	#message_runtime
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(
	object_localisation_node 
		src/object_localisation_node.cpp 
		src/ObjectLocalisation.cpp 
		src/filtering.cpp 
		src/meshing.cpp 
		src/keypoint.cpp
		src/feature2.hpp 
		src/rops_estimation.hpp
)

target_link_libraries(object_localisation_node ${catkin_LIBRARIES})
add_dependencies(object_localisation_node object_localisation_gencpp)
