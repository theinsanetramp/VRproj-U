cmake_minimum_required(VERSION 2.8.12)

project(CannyDetector_demo)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(CannyDetector_demo example.c)
target_link_libraries(CannyDetector_demo ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR}/libimagequant.a m)
