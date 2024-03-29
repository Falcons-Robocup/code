cmake_minimum_required(VERSION 3.5.1)

project(multiCamCalibrate)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

# Add link library and include directories for finding OpenCV
include_directories(/usr/include/opencv4)
link_directories(/usr/local/lib)

include_directories(./include)
include_directories(./../../packages/multiCam/include)
include_directories(./../../packages/facilities/common/include/ext)

include_directories(./../../vision/pylonToOpenCv)

include_directories(/opt/pylon/include)
link_directories(/opt/pylon/lib)

add_library(optiCalLib 
    src/optiCal.cpp
    src/optiCalGUI.cpp
    src/CameraCalibrator.cpp
    src/cvmatio.cpp
    src/multiCamVideoFeed.cpp
    ./../../packages/multiCam/src/camSysReceive.cpp
    ./../../vision/pylonToOpenCv/grabber.cpp
)

add_executable(optiCal src/optiCalMain.cpp)

target_link_libraries(optiCal 
    optiCalLib
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_calib3d
    opencv_videoio
    pthread
    FalconsCommon
    pylonbase
    pylonutility
    GenApi_gcc_v3_1_Basler_pylon
    GCBase_gcc_v3_1_Basler_pylon
)

add_executable(upgradeDewarpCalibration src/upgradeCalibration.cpp)

target_link_libraries(upgradeDewarpCalibration 
    optiCalLib
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_imgcodecs
    opencv_calib3d
    opencv_videoio
    pthread
    FalconsCommon
    pylonbase
    pylonutility
    GenApi_gcc_v3_1_Basler_pylon
    GCBase_gcc_v3_1_Basler_pylon
)

