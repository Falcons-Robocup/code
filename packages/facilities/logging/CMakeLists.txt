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

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

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

include_directories("inc")

rosbuild_add_library(logging src/cLogger.cpp src/cLogFilePlayback.cpp src/cLogPlayback.cpp src/cDbConnection.cpp src/cDbSync.cpp src/cLogFileWriter.cpp src/cLogFileReader.cpp src/cFrameBuffer.cpp)

rosbuild_add_executable(logger src/mLogger.cpp)
target_link_libraries(logger logging)

rosbuild_add_executable(rdlinfo src/mLogInfo.cpp)
target_link_libraries(rdlinfo logging)

rosbuild_add_executable(rdldump src/mLogDump.cpp)
target_link_libraries(rdldump logging)

rosbuild_add_executable(rdlfix src/mLogFix.cpp)
target_link_libraries(rdlfix logging)

rosbuild_add_executable(rtdbSync src/mRtdbSync.cpp)
target_link_libraries(rtdbSync logging)

rosbuild_add_pyunit(tst/testRdl.py TIMEOUT 10.0)

