cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(pcd_recognition)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -std=gnu++11")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_executable (pcd_recognition pcd_recognition.cpp)
target_link_libraries (pcd_recognition ${PCL_LIBRARIES})

