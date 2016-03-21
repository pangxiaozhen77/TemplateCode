cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pass_through_filter)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (pass_through_filter pass_through_filter.cpp)
target_link_libraries (pass_through_filter ${PCL_LIBRARIES})

