cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(diy_icp)

# edit path and version to match the pcl on your system
find_package(PCL 1.12.1 REQUIRED PATHS /usr/local/include/pcl-1.12)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (diy_icp src/diy_icp.cpp)
add_executable (txt2pcd src/txt2pcd.cpp)

target_link_libraries (diy_icp ${PCL_LIBRARIES})
target_link_libraries (txt2pcd ${PCL_LIBRARIES})
