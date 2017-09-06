cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

################################################################################
# BEGIN Override the buildin gtest stuff with our gmock/gtest package
# see also /opt/ros/jade/share/catkin/cmake/test/gtest.cmake
################################################################################
rosbuild_find_ros_package(gmock)
set(GTEST_LIBRARIES gmock)
include_directories(BEFORE ${gmock_PACKAGE_PATH}/include)
set(GTEST_INCLUDE_DIR ${gmock_PACKAGE_PATH}/include CACHE PATH rosoverride FORCE)
set(_CATKIN_GTEST_INCLUDE ${gmock_PACKAGE_PATH}/src/gtest/include/gtest/gtest.h CACHE FILEPATH rosoverride FORCE)
set(_CATKIN_GTEST_SRC ${gmock_PACKAGE_PATH}/src/gtest/src/gtest.cc CACHE FILEPATH rosoverride FORCE)
set(gtest_SOURCE_DIR ${gmock_PACKAGE_PATH}/src/gtest/src CACHE STATIC rosoverride FORCE)
set(GTEST_FROM_SOURCE_INCLUDE_DIRS ${gmock_PACKAGE_PATH}/src/gtest/include CACHE INTERNAL rosoverride FORCE)
################################################################################
# END Override the buildin gtest stuff with our gmock/gtest package
################################################################################
rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11")

rosbuild_add_library(refboxFeedback src/logger/cPacketRefboxLogger.cpp)
target_link_libraries(refboxFeedback json-c)

rosbuild_add_executable(refboxRelay src/refboxRelay.cpp src/cRosAdapterLogging.cpp src/TCPIP_client.cpp)
target_link_libraries(refboxRelay refboxFeedback)


#############################
# Testers
##############################
rosbuild_add_gtest(cPacketRefBoxLoggerTests tst/cPacketRefBoxLoggerTests.cpp TIMEOUT 10)
target_link_libraries(cPacketRefBoxLoggerTests refboxFeedback)

