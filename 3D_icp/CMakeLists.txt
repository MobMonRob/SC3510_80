cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(diy_icp)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (diy_icp src/diy_icp.cpp)
target_link_libraries (diy_icp ${PCL_LIBRARIES})