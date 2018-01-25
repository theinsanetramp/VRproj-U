cmake_minimum_required(VERSION 2.8.12)

project(moments)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(moments moments.cpp)
target_link_libraries(moments ${OpenCV_LIBS})
