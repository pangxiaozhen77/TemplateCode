cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(rops_feature_original)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (rops_feature_original rops_feature_original.cpp)
target_link_libraries (rops_feature_original ${PCL_LIBRARIES})

