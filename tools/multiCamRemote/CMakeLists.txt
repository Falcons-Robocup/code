cmake_minimum_required(VERSION 3.5.1)

project(multiCamRemote)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

# Add link library and include directories for finding OpenCV
include_directories(/usr/include/opencv4)
link_directories(/usr/local/lib)

include_directories(./include ./../../packages/multiCam/include $ENV{FALCONS_CODE_PATH}/packages/facilities/common/include/ext)
                                       
add_executable(multiCamRemote    src/multiCamRemote.cpp
                                 src/multicastReceive.cpp
                                 ./../../packages/multiCam/src/configurator.cpp
                                 ./../../packages/multiCam/src/robotFloor.cpp)

target_link_libraries(multiCamRemote
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_calib3d
    opencv_videoio
    pthread
    FalconsCommon
)
