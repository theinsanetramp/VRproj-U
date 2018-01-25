cmake_minimum_required(VERSION 2.8.12)

project(segment_objects)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(segment_objects segment_objects.cpp)
target_link_libraries(segment_objects ${OpenCV_LIBS})
