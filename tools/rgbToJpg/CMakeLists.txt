cmake_minimum_required(VERSION 3.5.1)

project(rgbToJpg)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

# Add link library and include directories for finding OpenCV
include_directories(/usr/include/opencv4)
link_directories(/usr/local/lib)
                                       
add_executable(rgbToJpg rgbToJpg.cpp)
target_link_libraries(rgbToJpg
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    pthread
)
