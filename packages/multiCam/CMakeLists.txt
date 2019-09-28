cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add link library for finding OpenCV libs
link_directories(/usr/local/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#Create libraries
rosbuild_add_library(mctypes src/types/robotLocationType.cpp
                           src/types/ballPositionType.cpp
                           src/types/obstaclePositionType.cpp)

# intended to be exported and used by raspi* packages
rosbuild_add_library(mcRaspiCore src/camSysReceive.cpp)

rosbuild_add_library(mcalgorithm src/multiCam.cpp
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
target_link_libraries(mcalgorithm mctypes)
target_link_libraries(mcalgorithm mcRaspiCore)
target_link_libraries(mcalgorithm opencv_core)
target_link_libraries(mcalgorithm opencv_imgproc)
target_link_libraries(mcalgorithm opencv_highgui)

rosbuild_add_library(mcobserver src/observer.cpp 
                                src/observerRtDB.cpp 
                              )

# Create executable for multiCam standalone package
rosbuild_add_executable(multiCamStandalone src/multiCamAlone.cpp)
target_link_libraries(multiCamStandalone mcalgorithm)
target_link_libraries(multiCamStandalone mctypes)   
target_link_libraries(multiCamStandalone mcobserver)
target_link_libraries(multiCamStandalone opencv_core)
target_link_libraries(multiCamStandalone opencv_imgproc)
rosbuild_link_boost(multiCamStandalone system thread)

# Create executable for ROS node
rosbuild_add_executable(multiCamNode src/multiCamRtDB.cpp)
target_link_libraries(multiCamNode mcalgorithm)
target_link_libraries(multiCamNode mctypes)
target_link_libraries(multiCamNode mcobserver)
target_link_libraries(multiCamNode opencv_core)
target_link_libraries(multiCamNode opencv_imgproc)
rosbuild_link_boost(multiCamNode system thread)

# Test executables
rosbuild_add_executable(dewarpCSV2BIN tst/dewarpCSV2BIN.cpp src/dewarp.cpp src/cvmatio.cpp)
target_link_libraries(dewarpCSV2BIN opencv_core)
target_link_libraries(dewarpCSV2BIN opencv_imgproc)
target_link_libraries(dewarpCSV2BIN opencv_highgui)
rosbuild_add_executable(testDewarp tst/testDewarp.cpp src/dewarp.cpp src/cvmatio.cpp)
target_link_libraries(testDewarp opencv_core)
target_link_libraries(testDewarp opencv_imgproc)
target_link_libraries(testDewarp opencv_highgui)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
