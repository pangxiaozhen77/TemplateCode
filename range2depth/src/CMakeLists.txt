cmake_minimum_required(VERSION 2.6 FATAL_ERROR)

project(range2depth)

find_package(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (range2depth range2depth.cpp)
target_link_libraries (range2depth ${PCL_LIBRARIES})
