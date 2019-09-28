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
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11")

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

rosbuild_add_library(mpClient src/adapters/cMotionPlanningClient.cpp)
target_link_libraries(mpClient wmClient)

rosbuild_add_library(mpAdapters    src/adapters/cRTDBInputAdapter.cpp
                                   src/adapters/cRTDBOutputAdapter.cpp)
#target_link_libraries(stores)

rosbuild_add_library(mpConfigAdapter  src/adapters/configuration/cConfigMotionPlanningData.cpp
                                      src/adapters/configuration/configuration.cpp
                                      src/stores/configurationStore.cpp)
                               
rosbuild_add_library(mpInterfaces  src/cPathPlanningInterface.cpp
                                   src/cBallHandlingInterface.cpp
                                   src/cQueryInterface.cpp)

rosbuild_add_library(mpTypes       src/types/cForbiddenAreaType.cpp)

rosbuild_add_library(mpActions     src/cAbstractAction.cpp
                                   src/cTimer.cpp
                                   src/algorithms/intercept.cpp
                                   src/cActionKick.cpp
                                   src/cActionShootAtTarget.cpp
                                   src/cActionPassToTarget.cpp
                                   src/cActionMoveToTarget.cpp
                                   src/cActionGetBall.cpp
                                   src/cActionInterceptBall.cpp
                                   src/cActionStop.cpp
                                   src/cActionTurnAwayFromOpponent.cpp
                                   src/cActionKeeperMove.cpp)

# main
rosbuild_add_executable(motionPlanner src/main_ros.cpp
                                      src/cMotionPlanner.cpp)
target_link_libraries(mpInterfaces mpTypes)
target_link_libraries(mpAdapters mpTypes)
target_link_libraries(motionPlanner mpInterfaces)
target_link_libraries(motionPlanner mpAdapters yaml-cpp)
target_link_libraries(motionPlanner mpTypes)
target_link_libraries(motionPlanner mpActions)
target_link_libraries(motionPlanner mpConfigAdapter)
rosbuild_link_boost(motionPlanner system thread)


# TESTERS
# cQueryInterface
rosbuild_add_gtest(cQueryInterfaceTest tst/cQueryInterfaceTest.cpp TIMEOUT 1)
target_link_libraries(cQueryInterfaceTest mpInterfaces)
