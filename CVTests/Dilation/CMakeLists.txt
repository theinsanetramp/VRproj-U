cmake_minimum_required(VERSION 2.8.12)

project(dilation)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(dilation dilation.cpp)
target_link_libraries(dilation ${OpenCV_LIBS})
