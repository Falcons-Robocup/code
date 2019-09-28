cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries

#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Add link library for finding OpenCV libs
link_directories(/usr/local/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -Wall -Werror")

rosbuild_add_library(tpactions   src/actions/cAbstractAction.cpp
                               src/actions/cActionSuccess.cpp
                               src/actions/cActionStop.cpp
                               src/actions/cActionShoot.cpp
                               src/actions/cActionPass.cpp
                               src/actions/cActionMove.cpp
                               src/actions/cActionPositionBeforePOI.cpp
                               src/actions/cActionPositionBehindPOI.cpp
                               src/actions/cActionPositionForOppSetpiece.cpp
                               src/actions/cActionPositionForOwnSetpiece.cpp
                               src/actions/cActionGetBall.cpp
                               src/actions/cActionInterceptBall.cpp
                               src/actions/cActionGoalKeeper.cpp
                               src/actions/cActionMoveToFreeSpot.cpp
                               src/actions/cActionAvoidPOI.cpp
                               src/actions/cActionDefendPenaltyArea.cpp
                               src/actions/cActionTurnAwayFromOpponent.cpp
                               src/actions/cActionDefendAttackingOpponent.cpp
                               src/actions/cActionDribbleForPass.cpp
                               src/actions/cActionDribbleForShot.cpp
                               )

target_link_libraries(tpactions yaml-cpp worldModelInterface tpworldStateFunctions tpstores rtdbOutputAdapter)

rosbuild_add_library(tprules     src/rules/ruleAvoidAreas.cpp
                               src/rules/ruleSetpieceExecuteFinished.cpp
                               src/rules/ruleStimulatePassing.cpp
                               src/utilities/timer.cpp
                               )

target_link_libraries(tprules tpstores)

rosbuild_add_library(tpconfigAdapter  src/adapters/configuration/cConfigHeightmaps.cpp
                                    src/adapters/configuration/cConfigInterceptBall.cpp
                                    src/adapters/configuration/cConfigRules.cpp
                                    src/adapters/configuration/cConfigShooting.cpp
                                    src/adapters/configuration/cConfigStrategy.cpp
                                    )

rosbuild_add_library(tpdecisionTree  src/cDecisionTree.cpp)
target_link_libraries(tpdecisionTree tpworldStateFunctions tpactions tpstores)

rosbuild_add_library(tpgameStateManager   src/gameStateManager.cpp
                                           src/gameStateTransitionTable.cpp
                                           src/types/gameState.cpp
                                           )
target_link_libraries(tpgameStateManager tprules tpstores yaml-cpp)

rosbuild_add_library(tpstores   src/heightmaps/abstractHeightMap.cpp
                              src/heightmaps/hmAvoidBall.cpp
                              src/heightmaps/hmAvoidObstacles.cpp
                              src/heightmaps/hmAvoidTeamMates.cpp
                              src/heightmaps/hmBetweenPoiAndClosestObstacle.cpp
                              src/heightmaps/hmCloseToBallClaimedLocation.cpp
                              src/heightmaps/hmCloseToOwnPos.cpp
                              src/heightmaps/hmInFrontOfOppGoal.cpp
                              src/heightmaps/hmNearObstacles.cpp
                              src/heightmaps/hmNearOwnGoal.cpp
                              src/heightmaps/hmNearPosition.cpp
                              src/heightmaps/hmObstaclesBlockingTeammates.cpp
                              src/heightmaps/hmObstaclesBlockingBall.cpp
                              src/heightmaps/hmObstaclesBlockingOppGoal.cpp
                              src/stores/ballStore.cpp
                              src/stores/configurationStore.cpp
                              src/stores/diagnosticsStore.cpp
                              src/stores/fieldDimensionsStore.cpp
                              src/stores/gameStateStore.cpp
                              src/stores/heightMapStore.cpp
                              src/stores/obstacleStore.cpp
                              src/stores/robotStore.cpp
                              src/types/ball.cpp
                              src/types/configuration.cpp
                              src/types/diagnostics.cpp
                              src/types/fieldDimensions.cpp
                              src/types/gameState.cpp
                              src/types/obstacle.cpp
                              src/types/robot.cpp
                              src/types/role.cpp
                              )
target_link_libraries(tpstores  Environment opencv_highgui rtdbOutputAdapter)

rosbuild_add_library(teamplayControlInterface src/cTeamplayControlInterface.cpp src/adapters/RtdbAdapterControlOverride.cpp)

rosbuild_add_library(worldModelInterface  src/cWorldModelInterface.cpp)
target_link_libraries(worldModelInterface tpstores)

rosbuild_add_library(objectPath  src/cObjectPath.cpp)
target_link_libraries(objectPath tpstores)

rosbuild_add_library(tpworldStateFunctions  src/cWorldStateFunctions.cpp)
target_link_libraries(tpworldStateFunctions worldModelInterface tprules tpstores rtdbInputAdapter)

rosbuild_add_library(rtdbInputAdapter  src/adapters/cRTDBInputAdapter.cpp)
target_link_libraries(rtdbInputAdapter tpstores mpClient worldModelInterface)

rosbuild_add_library(rtdbOutputAdapter  src/adapters/cRTDBOutputAdapter.cpp)

rosbuild_add_library(rtdbAdapterRefboxSignals src/adapters/cRtdbAdapterRefboxSignals.cpp)


# TESTERS
# WorldStateFunctions
rosbuild_add_gtest(cWorldStateFunctionsTest tst/cWorldStateFunctionsTest.cpp TIMEOUT 5)
target_link_libraries(cWorldStateFunctionsTest worldModelInterface tpworldStateFunctions Environment)

# ActionMove
#rosbuild_add_gtest(actionMoveTest tst/cActionMoveTest.cpp TIMEOUT 5)
#target_link_libraries(actionMoveTest gmock tpactions)

# ActionShoot
#rosbuild_add_gtest(actionShootTest tst/cActionShootTest.cpp TIMEOUT 5)
#target_link_libraries(actionShootTest gmock tpactions)

# ActionStop
#rosbuild_add_gtest(actionStopTest tst/cActionStopTest.cpp TIMEOUT 5)
#target_link_libraries(actionStopTest gmock tpactions)

# GameState Tester
rosbuild_add_gtest(gameStateTest tst/gameStateTest.cpp TIMEOUT 5)
target_link_libraries(gameStateTest tpgameStateManager)

# GameStateTransitionTable Tester
rosbuild_add_gtest(gameStateTransitionTableTest tst/gameStateTransitionTableTest.cpp TIMEOUT 5)
target_link_libraries(gameStateTransitionTableTest tpgameStateManager)

# AbstractHeightMap Tester
rosbuild_add_gtest(abstractHeightMapTest tst/abstractHeightMapTest.cpp TIMEOUT 1)
target_link_libraries(abstractHeightMapTest tpstores)

# HeightMapStore Tester
rosbuild_add_gtest(heightMapStoreTest tst/heightMapStoreTest.cpp TIMEOUT 1)
target_link_libraries(heightMapStoreTest tpstores)

# HeightMap hmBetweenPoiAndClosestObstacle Tester
rosbuild_add_gtest(hmBetweenPoiAndClosestObstacleTest tst/hmBetweenPoiAndClosestObstacleTest.cpp TIMEOUT 1)
target_link_libraries(hmBetweenPoiAndClosestObstacleTest tpstores)

# HeightMap hmInFrontOfOppGoal Tester
rosbuild_add_gtest(hmInFrontOfOppGoalTest tst/hmInFrontOfOppGoalTest.cpp TIMEOUT 1)
target_link_libraries(hmInFrontOfOppGoalTest tpstores)

# HeightMap hmNearPosition Tester
rosbuild_add_gtest(hmNearPositionTest tst/hmNearPositionTest.cpp TIMEOUT 1)
target_link_libraries(hmNearPositionTest tpstores)

# ObjectPath Tester
rosbuild_add_gtest(cObjectPathTest tst/cObjectPathTest.cpp TIMEOUT 1)
target_link_libraries(cObjectPathTest objectPath)

# ObstacleStore Tester
rosbuild_add_gtest(obstacleStoreTest tst/obstacleStoreTest.cpp TIMEOUT 1)
target_link_libraries(obstacleStoreTest tpstores)

# RobotStore Tester
rosbuild_add_gtest(robotStoreTest tst/robotStoreTest.cpp TIMEOUT 1)
target_link_libraries(robotStoreTest tpstores)

# RuleAvoidAreas Tester
rosbuild_add_gtest(ruleAvoidAreasTest tst/ruleAvoidAreasTest.cpp TIMEOUT 1)
target_link_libraries(ruleAvoidAreasTest gmock tprules tpstores)

# RuleSetPieceExecuteFinished Tester
rosbuild_add_gtest(ruleSetpieceExecuteFinishedTest tst/ruleSetpieceExecuteFinishedTest.cpp TIMEOUT 1)
target_link_libraries(ruleSetpieceExecuteFinishedTest gmock tprules tpstores)

# RuleStimulatePassing Tester
rosbuild_add_gtest(ruleStimulatePassingTest tst/ruleStimulatePassingTest.cpp TIMEOUT 1)
target_link_libraries(ruleStimulatePassingTest tprules tpstores)

# Timer tester
rosbuild_add_gtest(timerTest tst/timerTest.cpp src/utilities/timer.cpp TIMEOUT 1)
target_link_libraries(timerTest)
rosbuild_link_boost(timerTest system filesystem)

# Tree tester
rosbuild_add_gtest(cTreeTesterTest tst/cTreeTester.cpp TIMEOUT 5)
target_link_libraries(cTreeTesterTest tpdecisionTree tpworldStateFunctions)
rosbuild_link_boost(cTreeTesterTest system)

# TOOLS
# TODO: need to be adapted to RTDB
#rosbuild_add_executable(liveTuningTool  tools/liveTuningTool.cpp)
#target_link_libraries(liveTuningTool    tpconfigAdapter tpstores)
#rosbuild_link_boost(liveTuningTool      system)

#rosbuild_add_executable(standaloneTuningTool  tools/standaloneTuningTool.cpp)
#target_link_libraries(standaloneTuningTool    tpconfigAdapter tpstores)
#rosbuild_link_boost(standaloneTuningTool      system)

# MAIN 
rosbuild_add_executable(teamplay_main  src/main.cpp)
target_link_libraries(teamplay_main tpconfigAdapter
                                    rtdbInputAdapter
                                    rtdbOutputAdapter
                                    teamplayControlInterface
                                    worldModelInterface 
                                    rtdbAdapterRefboxSignals 
                                    tpgameStateManager 
                                    tpdecisionTree
                                    tpactions)
rosbuild_link_boost(teamplay_main system thread)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
