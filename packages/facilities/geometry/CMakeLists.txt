cmake_minimum_required(VERSION 3.5.1)

project(Geometry)

find_package(Facilities)

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

include_directories("include/ext" "include")
include_directories("/usr/include/opencv4")

#Create library falconsGeometry
add_library(falconsGeometry
                src/intersect.cpp
                src/linalgcv.cpp
                src/linepoint2D.cpp
                src/polygon2D.cpp
                )
target_link_libraries(falconsGeometry opencv_core Tracing)


