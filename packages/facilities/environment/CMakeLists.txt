cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

#Set compiler to C++11 for jsoncpp
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")
else()
        message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support.
 Please use a different C++ compiler.")
endif()
####  END C++11 ####

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

#add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()


#Create library libEnvironmentCommon
rosbuild_add_library(EnvironmentCommon src/cEnvironmentCommon.cpp)
target_link_libraries(EnvironmentCommon JSON)

#Create library libEnvironmentField
rosbuild_add_library(EnvironmentField src/cEnvironmentField.cpp)
target_link_libraries(EnvironmentField EnvironmentCommon)

#Create library libEnvironmentBall
rosbuild_add_library(EnvironmentBall src/cEnvironmentBall.cpp)
target_link_libraries(EnvironmentBall EnvironmentCommon)

#Create library libEnvironmentRobot
rosbuild_add_library(EnvironmentRobot src/cEnvironmentRobot.cpp)
target_link_libraries(EnvironmentRobot EnvironmentCommon)

#Create library libEnvironment
rosbuild_add_library(Environment src/cEnvironmentCommon.cpp src/cEnvironmentField.cpp src/cEnvironmentBall.cpp src/cEnvironmentRobot.cpp)

#Testers
rosbuild_add_gtest( cEnvironmentTest tst/cEnvironmentTest.cpp )

#Target link libraries
target_link_libraries( cEnvironmentTest EnvironmentCommon EnvironmentField )

#Get Python package info to build the python modules
#include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
#pkg_check_modules( PYTHON REQUIRED python2)
#include_directories( ${PYTHON_INCLUDE_DIRS} )
#rosbuild_add_library( _mymodule pymodule/environment_wrap.cxx)

add_subdirectory(pymodule)

# Create autotester for compass package


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
