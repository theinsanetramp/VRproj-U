cmake_minimum_required(VERSION 2.8.12)

project(CannyDetector_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(CannyDetector_demo CannyDetector_demo.cpp sobol.cpp lodepng.cpp compressor.cpp FloodFill.cpp)
target_link_libraries(CannyDetector_demo ${OpenCV_LIBS} ${CMAKE_SOURCE_DIR}/libimagequant.a m)
