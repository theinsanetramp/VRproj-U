cmake_minimum_required(VERSION 2.8.12)

project(potracelib_demo)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(potracelib_demo potracelib_demo.c potracelib.o)
target_link_libraries(potracelib_demo ${OpenCV_LIBS})
