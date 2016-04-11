cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(matching_eval)

find_package(PCL 1.7 REQUIRED)

include_directories(. ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (
matching_eval 
	matching_eval 
	feature2.hpp 
	rops_estimation.hpp
)
target_link_libraries (matching_eval ${PCL_LIBRARIES})
