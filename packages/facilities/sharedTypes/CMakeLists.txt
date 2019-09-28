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

# using shared_ptr
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

include_directories("include")

#Create library libFalconsTypes
rosbuild_add_library(FalconsTypes src/objectMeasurement.cpp src/uniqueObjectID.cpp)

#Testers

add_custom_target(
    run ALL
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/generateEnums.py -v -i ${CMAKE_CURRENT_SOURCE_DIR}/include -o ${CMAKE_CURRENT_SOURCE_DIR}/include/generated_enum2str.hpp -p ${CMAKE_CURRENT_SOURCE_DIR}/src/sharedTypes.py
    BYPRODUCTS generated_enum2str.hpp sharedTypes.py
    COMMENT "Generating code from enums"
)

add_dependencies(FalconsTypes run)

