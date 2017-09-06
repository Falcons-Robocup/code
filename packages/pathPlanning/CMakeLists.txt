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

# TKOV: enable enumerate value to be scoped
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

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

##### OBSOLETE #####
#rosbuild_add_executable(PFM_with_MC src/PFM_with_movement_constraints.cpp)
#rosbuild_add_executable(set_obstacles src/set_obstacles.cpp)
####################

rosbuild_add_library(PathPlanningRosStub src/cPathPlanningRosStub.cpp)

rosbuild_add_library(pathPlanningFacilities src/facilities/cObstacleFacilities.cpp)
rosbuild_add_library(rosConnect src/adapters/cWorldModelAdapter.cpp
                                src/adapters/cTeamplayAdapter.cpp
                                src/adapters/cSetSpeedAdapter.cpp
                                src/adapters/cReconfigureAdapter.cpp
                                src/adapters/cDiagnosticsAdapter.cpp)
rosbuild_add_library(pathPlanningAlgorithms src/cAbstractPathPlanning.cpp
                                            src/algorithms/cXYLinear.cpp
                                            src/algorithms/cPhiLinear.cpp
                                            src/algorithms/cXYLinearAcceleration.cpp
                                            src/algorithms/cPhiLinearAcceleration.cpp
                                            src/algorithms/cXYPFM.cpp
                                            src/algorithms/cPhiPFM.cpp
                                            src/algorithms/cXYTrajectory.cpp
                                            src/algorithms/cXYPID.cpp
                                            src/algorithms/cXPID.cpp
                                            src/algorithms/cYPID.cpp
                                            src/algorithms/cPhiPID.cpp
                                            src/algorithms/cXYBrake.cpp
                                            src/algorithms/cXYTurnThenMove.cpp
                                            src/algorithms/cPhiMoveThenTurn.cpp
                                            src/algorithms/cMoveAtSpeed.cpp
                                            src/algorithms/cTokyoDrift.cpp
                                            src/algorithms/cTokyoDriftVelocity.cpp
                                            src/algorithms/cFCStoRCS.cpp
                                            src/algorithms/cBoundaryLimiter.cpp
                                            src/algorithms/cLimitVelocities.cpp)
target_link_libraries(pathPlanningAlgorithms rosConnect pathPlanningFacilities)
rosbuild_add_executable(pathplanningNode src/main_ros.cpp
                                         src/cPathPlanningMain.cpp
                                         src/cPathPlanningData.cpp)
target_link_libraries(pathplanningNode rosConnect yaml-cpp)
target_link_libraries(pathplanningNode pathPlanningAlgorithms)


