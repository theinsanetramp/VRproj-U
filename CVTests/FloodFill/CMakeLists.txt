cmake_minimum_required(VERSION 2.8.12)

project(FloodFill)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(FloodFill FloodFill.cpp)
target_link_libraries(FloodFill ${OpenCV_LIBS})
