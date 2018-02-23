cmake_minimum_required(VERSION 2.8.12)

project(CannyDetector_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -LLIBDIR -Wl,-rpath -Wl,LIBDIR")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(example example.c lodepng.c)
target_link_libraries(example ${OpenCV_LIBS} ${CMAKE_CURRENT_LIST_DIR}/libimagequant.a)
