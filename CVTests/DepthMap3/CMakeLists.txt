cmake_minimum_required(VERSION 2.8.12)

project(stereoBM)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(stereoBM stereoBM.cpp elas.cpp descriptor.cpp filter.cpp matrix.cpp triangle.cpp)
target_link_libraries(stereoBM ${OpenCV_LIBS})
