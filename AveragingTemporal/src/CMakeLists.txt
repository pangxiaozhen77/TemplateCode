cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(averaging_temporal)

find_package(PCL 1.7 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (averaging_temporal averaging_temporal.cpp)
target_link_libraries (averaging_temporal ${PCL_LIBRARIES})

