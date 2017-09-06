cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11")

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

include_directories("include/ext")

#Create library libFalconsCommon
rosbuild_add_library(FalconsCommon src/cFalconsCommon.cpp 
                                   src/fsm.cpp 
                                   src/cLinearRegression.cpp
                                   src/timeConvert.cpp
                                   src/tracer.cpp 
                                   src/eventType.cpp
                                   )

# timeConvert utility
rosbuild_add_executable(timeConvert src/timeConvertMain.cpp)
target_link_libraries(timeConvert FalconsCommon)

#Testers
rosbuild_add_gtest(testPose2D tst/testPose2D.cpp TIMEOUT 1)
rosbuild_add_gtest(testVelocity2D tst/testVelocity2D.cpp TIMEOUT 1)

# Create autotester for compass package
#rosbuild_add_pyunit(tst/autotester.py)


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
