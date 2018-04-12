cmake_minimum_required(VERSION 2.8.12)

project(Test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -pthread")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(Test Test.cpp)
target_link_libraries(Test ${OpenCV_LIBS})
