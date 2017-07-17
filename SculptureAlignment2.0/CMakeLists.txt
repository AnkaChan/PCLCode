cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(LUM_Correct2)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (LUM_Correct2 LUM_Correct2.cpp)
target_link_libraries (LUM_Correct2 ${PCL_LIBRARIES})