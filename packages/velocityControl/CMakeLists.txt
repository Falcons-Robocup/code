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

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()


#add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

# ROS adapters
rosbuild_add_library(vcRtdbAdapters src/adapters/cRTDBInputAdapter.cpp
                                    src/adapters/cRTDBOutputAdapter.cpp)

rosbuild_add_library(velocityControlAlgorithms src/cAbstractVelocityControl.cpp
                                               src/algorithms/cTranslateFeedback.cpp
                                               src/algorithms/cPublishFeedback.cpp
                                               src/algorithms/cTranslateTarget.cpp
                                               src/algorithms/cPublishTarget.cpp)

# main
rosbuild_add_executable(velocityControl src/main_ros.cpp
                                        src/cVelocityControlMain.cpp
                                        src/cVelocityControlData.cpp)

# linking
target_link_libraries(velocityControl vcRtdbAdapters)
target_link_libraries(velocityControl velocityControlAlgorithms)
rosbuild_link_boost(velocityControl system thread)
