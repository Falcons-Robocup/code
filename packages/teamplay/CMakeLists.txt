cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)


# Replace catkin variable

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
rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11")

rosbuild_add_library(actions   src/actions/cAbstractAction.cpp
                               src/actions/cActionSuccess.cpp
                               src/actions/cActionStop.cpp
                               src/actions/cActionShoot.cpp
                               src/actions/cActionMove.cpp
                               src/actions/cActionMoveToPenaltyAngle.cpp
                               src/actions/cActionPositionBeforePOI.cpp
                               src/actions/cActionPositionBehindPOI.cpp
                               src/actions/cActionFaceNearestTeammember.cpp
                               src/actions/cActionGetBall.cpp
                               src/actions/cActionInterceptBall.cpp
                               src/actions/cActionGoalKeeper.cpp
                               src/actions/cActionMoveToFreeSpot.cpp
                               src/actions/cActionAvoidPOI.cpp
                               src/actions/cActionGetBallOnVector.cpp
                               src/actions/cActionLongTurnToGoal.cpp
                               src/actions/cActionAimForShotOnGoal.cpp
                               src/actions/cActionDefendAssist.cpp
                               )
target_link_libraries(actions teamplaySync)                               

target_link_libraries(actions yaml-cpp motionInterfaces worldModelInterface worldStateFunctions algorithms rosAdapterWorldModel stores)

rosbuild_add_library(algorithms  src/algorithms/cPFM.cpp
                                 )                            
target_link_libraries(algorithms  trace)

rosbuild_add_library(rules     src/rules/ruleAvoidAreas.cpp
                               src/rules/ruleSetpieceExecuteFinished.cpp
                               src/rules/ruleStimulatePassing.cpp
                               src/utilities/timer.cpp
                               )

target_link_libraries(rules stores)

rosbuild_add_library(configAdapter  src/adapters/configuration/cConfigInterceptBall.cpp
                                    src/adapters/configuration/cConfigRules.cpp
                                    src/adapters/configuration/cConfigShooting.cpp
                                    src/adapters/configuration/cConfigStrategy.cpp
                                    )

rosbuild_add_library(decisionTree  src/cDecisionTree.cpp)
target_link_libraries(decisionTree worldStateFunctions actions stores)

rosbuild_add_library(gameStateManager   src/gameStateManager.cpp
                                           src/gameStateTransitionTable.cpp
                                           src/types/gameState.cpp
                                           )
target_link_libraries(gameStateManager rules stores trace yaml-cpp)

rosbuild_add_library(motionInterfaces  src/adapters/cHALInterface.cpp
                                       src/adapters/cPathPlanningInterface.cpp
                                       src/adapters/cShootPlanningInterface.cpp
                                       )

rosbuild_add_library(stores   src/stores/ballStore.cpp
                              src/stores/configurationStore.cpp
                              src/stores/diagnosticsStore.cpp
                              src/stores/fieldDimensionsStore.cpp
                              src/stores/gameStateStore.cpp
                              src/stores/ownRobotStore.cpp
                              src/stores/teamMatesStore.cpp
                              src/types/ball.cpp
                              src/types/configuration.cpp
                              src/types/diagnostics.cpp
                              src/types/fieldDimensions.cpp
                              src/types/gameState.cpp
                              src/types/robot.cpp
                              src/types/teamMates.cpp
                              )
target_link_libraries(stores  Environment trace teamplaySync)

rosbuild_add_library(teammateInterface  src/cTeammateInterface.cpp)

rosbuild_add_library(teamplayCommon  src/cTeamplayCommon.cpp)

rosbuild_add_library(teamplayControlInterface src/cTeamplayControlInterface.cpp src/adapters/cControlInterfaceROS.cpp)

rosbuild_add_library(trace  src/utilities/trace.cpp)

rosbuild_add_library(worldModelInterface  src/cWorldModelInterface.cpp)
target_link_libraries(worldModelInterface stores trace)

rosbuild_add_library(objectPath  src/cObjectPath.cpp)
target_link_libraries(objectPath stores)

rosbuild_add_library(worldStateFunctions  src/cWorldStateFunctions.cpp)
target_link_libraries(worldStateFunctions worldModelInterface objectPath rules teamplayCommon stores)

rosbuild_add_library(rosAdapterWorldModel  src/adapters/cRosAdapterWorldModel.cpp)
target_link_libraries(rosAdapterWorldModel stores)

rosbuild_add_library(udpAdapterRefboxSignals src/adapters/cUdpAdapterRefboxSignals.cpp)
target_link_libraries(udpAdapterRefboxSignals Network)

rosbuild_add_library(teamplaySync   src/types/intentionSync.cpp                              
                                    )
target_link_libraries(teamplaySync  Network)

# TESTERS
# WorldStateFunctions
rosbuild_add_gtest(cWorldStateFunctionsTest tst/cWorldStateFunctionsTest.cpp TIMEOUT 10)
target_link_libraries(cWorldStateFunctionsTest gmock worldModelInterface worldStateFunctions Environment)

# WorldStateFunctionsNew: will eventually replace WorldStateFunctions. Work in progress (by Coen)
rosbuild_add_gtest(cWorldStateFunctionsTestNew tst/cWorldStateFunctionsTestNew.cpp TIMEOUT 1)
target_link_libraries(cWorldStateFunctionsTestNew gmock worldStateFunctions)

# ActionMove
rosbuild_add_gtest(actionMoveTest tst/cActionMoveTest.cpp TIMEOUT 5)
target_link_libraries(actionMoveTest gmock actions)

# ActionShoot
rosbuild_add_gtest(actionShootTest tst/cActionShootTest.cpp TIMEOUT 5)
target_link_libraries(actionShootTest gmock actions trace)

# ActionStop
rosbuild_add_gtest(actionStopTest tst/cActionStopTest.cpp TIMEOUT 5)
target_link_libraries(actionStopTest gmock actions worldStateFunctions trace)

# PathPlanningInterface
# disabled because it requires roscore
#rosbuild_add_gtest(pathPlanningInterfaceTest tst/cPathPlanningInterfaceTest.cpp TIMEOUT 5)
#target_link_libraries(pathPlanningInterfaceTest motionInterfaces trace)

# ShootPlanningInterface
# disabled because it requires roscore
#rosbuild_add_gtest(shootPlanningInterfaceTest tst/cShootPlanningInterfaceTest.cpp TIMEOUT 5)
#target_link_libraries(shootPlanningInterfaceTest motionInterfaces trace)

# WorldModelInterface
# disabled because it requires roscore
#rosbuild_add_gtest(cWorldModelInterfaceTest tst/cWorldModelInterfaceTest.cpp TIMEOUT 5)
#target_link_libraries(cWorldModelInterfaceTest rosAdapterWorldModel worldModelInterface trace)

# GameState Tester
rosbuild_add_gtest(gameStateTest tst/gameStateTest.cpp TIMEOUT 5)
target_link_libraries(gameStateTest gameStateManager trace)

# GameStateTransitionTable Tester
rosbuild_add_gtest(gameStateTransitionTableTest tst/gameStateTransitionTableTest.cpp TIMEOUT 5)
target_link_libraries(gameStateTransitionTableTest gameStateManager trace)

# ObjectPath manager
rosbuild_add_gtest(cObjectPathTest tst/cObjectPathTest.cpp TIMEOUT 5)
target_link_libraries(cObjectPathTest objectPath trace)

# PFM Tester
rosbuild_add_gtest(algorithmsTest tst/cPFMTest.cpp TIMEOUT 5)
target_link_libraries(algorithmsTest algorithms trace)

# RuleAvoidAreas Tester
rosbuild_add_gtest(ruleAvoidAreasTest tst/ruleAvoidAreasTest.cpp TIMEOUT 1)
target_link_libraries(ruleAvoidAreasTest gmock rules stores trace)

# RuleSetPieceExecuteFinished Tester
rosbuild_add_gtest(ruleSetpieceExecuteFinishedTest tst/ruleSetpieceExecuteFinishedTest.cpp TIMEOUT 1)
target_link_libraries(ruleSetpieceExecuteFinishedTest gmock rules stores trace)

# RuleStimulatePassing Tester
rosbuild_add_gtest(ruleStimulatePassingTest tst/ruleStimulatePassingTest.cpp TIMEOUT 1)
target_link_libraries(ruleStimulatePassingTest gmock rules stores trace)

# Timer tester
rosbuild_add_gtest(timerTest tst/timerTest.cpp src/utilities/timer.cpp TIMEOUT 1)
target_link_libraries(timerTest trace)

# Tree tester
rosbuild_add_gtest(cTreeTesterTest tst/cTreeTester.cpp TIMEOUT 5)
target_link_libraries(cTreeTesterTest decisionTree worldStateFunctions trace)

# MAIN 
rosbuild_add_executable(teamplay_main  src/main.cpp)
target_link_libraries(teamplay_main configAdapter
                                    rosAdapterWorldModel
                                    motionInterfaces
                                    teamplayControlInterface
                                    worldModelInterface 
                                    udpAdapterRefboxSignals 
                                    gameStateManager 
                                    decisionTree
                                    Network
                                    actions
                                    trace
                                    teamplaySync)


rosbuild_add_executable(teamplay_coach src/main_coach.cpp)
target_link_libraries( teamplay_coach teamplayControlInterface 
									trace decisionTree )

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
