cmake_minimum_required(VERSION 3.5.1)

project(multiCam)

find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

# Add link library and include directories for finding OpenCV
include_directories(/usr/include/opencv4)
link_directories(/usr/local/lib)
# include path
include_directories(include/)

#Create libraries
add_library(mctypes src/types/robotLocationType.cpp
                           src/types/ballPositionType.cpp
                           src/types/obstaclePositionType.cpp)

# intended to be exported and used by raspi* packages
add_library(mcRaspiCore src/camSysReceive.cpp)
target_link_libraries(mcRaspiCore opencv_core opencv_imgproc opencv_highgui opencv_imgcodecs)

add_library(mcalgorithm
    src/multiCam.cpp
    src/BallDistanceEstimator.cpp
    src/ballDetection.cpp
    src/obstacleDetection.cpp
    src/determinePosition.cpp
    src/cvmatio.cpp
    src/dewarp.cpp
    src/robotFloor.cpp
    src/fieldLut.cpp
    src/linePointDetection.cpp
    src/localization.cpp
    src/viewer.cpp
    src/multicastSend.cpp
    src/configurator.cpp
    src/preprocessor.cpp
    src/cameraControl.cpp
    src/cameraReceive.cpp
    src/simplex.cpp
)

target_link_libraries(mcalgorithm
    mctypes
    mcRaspiCore
    opencv_core
    opencv_imgproc
    opencv_highgui
    opencv_videoio
    pthread
    filters
    FalconsTypes
    FalconsCommon
)
if(DEFINED ENV{OPENCV_VERSION})
    if(NOT $ENV{OPENCV_VERSION} LESS 3)
        target_link_libraries(mcalgorithm opencv_videoio)
    endif()
endif()

add_library(mcobserver  src/observer.cpp  src/observerRtDB.cpp)
target_link_libraries(mcobserver
    FalconsCommon
    FalconsRTDB
    diagnostics
    )

# Create executable for multiCam standalone package
add_executable(multiCamStandalone src/multiCamAlone.cpp)
target_link_libraries(multiCamStandalone
    mcalgorithm
    mctypes
    mcobserver
    opencv_core
    opencv_imgproc
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
)

# Create executable
add_executable(multiCamNode src/multiCamRtDB.cpp)
target_link_libraries(multiCamNode
    mcalgorithm
    mctypes
    mcobserver
    opencv_core
    opencv_imgproc
    ${Boost_SYSTEM_LIBRARY}
    ${Boost_THREAD_LIBRARY}
    Tracing
)

# Test executables
add_executable(testDewarp tst/testDewarp.cpp src/dewarp.cpp src/cvmatio.cpp)
target_link_libraries(testDewarp
    opencv_core
    opencv_imgproc
    opencv_imgcodecs
    opencv_highgui
    FalconsCommon
)
if(DEFINED ENV{OPENCV_VERSION})
    if(NOT $ENV{OPENCV_VERSION} LESS 3)
        target_link_libraries(testDewarp opencv_imgcodecs)
    endif()
endif()


