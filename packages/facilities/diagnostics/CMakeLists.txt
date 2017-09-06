cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

rosbuild_init()
find_package(PythonLibs REQUIRED)

# TKOV: enable enumerate value to be scoped
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

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

# Create library libDiagnostics
rosbuild_add_library(diagnostics src/cDiagnosticsReceiver.cpp
                                 src/cDiagnosticsEvents.cpp
                                 src/cDiagnosticsDutyCycle.cpp
                                 )

# Create executable for coach listener
rosbuild_add_executable(diagCoachListener src/diagCoachListener.cpp)
target_link_libraries(diagCoachListener diagnostics)

# Add stress tester
#rosbuild_add_rostest(tst/stress_test.xml) # not used anymore
rosbuild_add_executable(testInfoError tst/testInfoError.cpp)
target_link_libraries(testInfoError diagnostics)

