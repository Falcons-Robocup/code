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
rosbuild_add_library(types src/types/robotLocationType.cpp
                           src/types/ballPositionType.cpp
                           src/types/obstaclePositionType.cpp)
                           
rosbuild_add_library(algorithm src/vision.cpp
                               src/ballDetection.cpp
                               src/ballDetection.cpp
                               src/ballPossession.cpp
                               src/cFrameDiagnostics.cpp
                               src/obstacleDetection.cpp
                               src/determinePosition.cpp
                               src/robotFloor.cpp
                               src/fieldLut.cpp
                               src/linePointDetection.cpp
                               src/localization.cpp
                               src/viewer.cpp
                               src/multicastSend.cpp
                               src/configurator.cpp
                               src/preprocessor.cpp
                               src/cameraControl.cpp
                               src/simplex.cpp
                               )             
target_link_libraries(algorithm types)                                     
target_link_libraries(algorithm opencv_core)
target_link_libraries(algorithm opencv_imgproc)
target_link_libraries(algorithm opencv_highgui)

rosbuild_add_library(observer src/observer.cpp 
                              src/observerRos.cpp 
                              )

# Create executable for vision standalone package
rosbuild_add_executable(visionStandalone src/visionAlone.cpp)
target_link_libraries(visionStandalone algorithm)
target_link_libraries(visionStandalone types)   
target_link_libraries(visionStandalone observer)
target_link_libraries(visionStandalone opencv_core)
target_link_libraries(visionStandalone opencv_imgproc)

# Create executable for ROS node
rosbuild_add_executable(visionNode src/visionRos.cpp)
target_link_libraries(visionNode algorithm)
target_link_libraries(visionNode types)
target_link_libraries(visionNode observer)
target_link_libraries(visionNode opencv_core)
target_link_libraries(visionNode opencv_imgproc)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
