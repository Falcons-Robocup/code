cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -Wall -Werror")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

# libraries
rosbuild_add_library(bhController  src/ballHandlingControl.cpp)

rosbuild_add_library(bhRtdbAdapters  src/adapters/cRTDBInputAdapter.cpp
                                     src/adapters/cRTDBOutputAdapter.cpp
                                     src/adapters/cReconfigureAdapter.cpp)

# main
rosbuild_add_executable(ballHandling src/main_ros.cpp)

target_link_libraries(ballHandling bhController bhRtdbAdapters yaml-cpp)
rosbuild_link_boost(ballHandling system thread)

