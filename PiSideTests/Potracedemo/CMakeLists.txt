cmake_minimum_required(VERSION 2.8.12)

project(potracelib_demo)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -LLIBDIR -Wl,-rpath -Wl,LIBDIR --no-as-needed")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(potracelib_demo potracelib_demo.c )
target_link_libraries(potracelib_demo ${OpenCV_LIBS} /usr/local/lib/libpotrace.a m)
