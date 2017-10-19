cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(ZeroFilters)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (ZeroFilters ZeroFilters.cpp)
target_link_libraries (ZeroFilters ${PCL_LIBRARIES})