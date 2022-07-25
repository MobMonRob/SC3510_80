cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(bspline_fitting)

find_package(PCL 1.12.1 EXACT REQUIRED PATHS /usr/local/include/pcl-1.8)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (bspline_fitting bspline_fitting.cpp)
target_link_libraries (bspline_fitting ${PCL_LIBRARIES})