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

# Need C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x -Wall -Werror")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

rosbuild_add_library(spshootAdapters src/adapters/cRTDBInputAdapter.cpp
                                     src/adapters/cRTDBOutputAdapter.cpp
                                     src/cReconfigureAdapter.cpp)

rosbuild_add_executable(shoot_planner src/main_ros.cpp
                                      src/cLinearInterpolator.cpp
                                      src/cInterpolator.cpp
                                      src/cShotCalibration.cpp
                                      src/cShotSolver.cpp
                                      src/cShootPlanner.cpp)
target_link_libraries(shoot_planner spshootAdapters yaml-cpp)
rosbuild_link_boost(shoot_planner system thread)


rosbuild_add_executable(dumpTables  tst/dumpTables.cpp
                                      src/cLinearInterpolator.cpp
                                      src/cInterpolator.cpp
                                      src/cShotCalibration.cpp
                                      src/cShotSolver.cpp)


