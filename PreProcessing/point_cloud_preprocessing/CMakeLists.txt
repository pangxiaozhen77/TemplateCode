cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_preprocessing)

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
  	point_cloud_preprocessing
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


add_executable(point_cloud_preprocessing_node src/point_cloud_preprocessing_node.cpp src/PointCloudPreprocessing.cpp src/filtering.cpp)
target_link_libraries(point_cloud_preprocessing_node ${catkin_LIBRARIES})
add_dependencies(point_cloud_preprocessing_node point_cloud_preprocessing_gencpp)
