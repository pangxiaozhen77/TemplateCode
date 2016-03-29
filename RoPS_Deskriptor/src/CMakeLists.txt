cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rops_descriptor)

find_package(PCL 1.7 REQUIRED)

include_directories(. ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rops_descriptor rops_descriptor.cpp feature.hpp rops_estimation.hpp)
target_link_libraries (rops_descriptor ${PCL_LIBRARIES})
