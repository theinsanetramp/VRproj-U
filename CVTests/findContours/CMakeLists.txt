cmake_minimum_required(VERSION 2.8.12)

project(findContours_demo)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(findContours_demo findContours_demo.cpp)
target_link_libraries(findContours_demo ${OpenCV_LIBS})
