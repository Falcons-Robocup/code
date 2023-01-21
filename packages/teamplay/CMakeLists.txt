cmake_minimum_required(VERSION 3.5.1)

project(Teamplay)

find_package(Boost COMPONENTS system thread filesystem REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)
find_package(MotionPlanning)
find_package(PathPlanning)
find_package(VelocityControl)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

# Add link library for finding OpenCV libs
link_directories(/usr/local/lib)

add_library(tpactions   src/actions/cAbstractAction.cpp
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
                        src/actions/cActionDribble.cpp
                        )

target_link_libraries(tpactions yaml-cpp worldModelInterface tpworldStateFunctions tpstores rtdbOutputAdapter)

add_library(tprules     src/rules/ruleAvoidAreas.cpp
                        src/rules/ruleSetpieceExecuteFinished.cpp
                        src/rules/ruleStimulatePassing.cpp
                        src/utilities/timer.cpp
                        )

# RtDButils for rtime (timer.cpp)
target_link_libraries(tprules tpstores Tracing falconsGeometry RtDButils FTime)

add_library(tpconfigAdapter src/adapters/ConfigAdapter.cpp)
target_link_libraries(tpconfigAdapter diagnostics)

add_library(tpdecisionTree  src/cDecisionTree.cpp)
target_link_libraries(tpdecisionTree tpworldStateFunctions tpactions tpstores)

add_library(tpgameStateManager   src/gameStateManager.cpp
                                 src/gameStateTransitionTable.cpp
                                 src/types/gameState.cpp
                                 )
target_link_libraries(tpgameStateManager tprules tpstores yaml-cpp)

add_library(tpstores   src/heightmaps/abstractHeightMap.cpp
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
target_link_libraries(tpstores
                      Environment
                      opencv_highgui
                      rtdbOutputAdapter
                      diagnostics
                      falconsGeometry)
if(DEFINED ENV{OPENCV_VERSION})
    if(NOT $ENV{OPENCV_VERSION} LESS 3)
        target_link_libraries(tpstores opencv_imgcodecs)
    endif()
endif()

add_library(teamplayControlInterface src/cTeamplayControlInterface.cpp src/adapters/RtdbAdapterControlOverride.cpp)
target_link_libraries(teamplayControlInterface FalconsTypes)

add_library(worldModelInterface  src/cWorldModelInterface.cpp)
target_link_libraries(worldModelInterface tpstores tprules)

add_library(objectPath  src/cObjectPath.cpp)
target_link_libraries(objectPath tpstores)

add_library(tpworldStateFunctions  src/cWorldStateFunctions.cpp)
target_link_libraries(tpworldStateFunctions worldModelInterface tprules tpstores rtdbInputAdapter)

add_library(rtdbInputAdapter  src/adapters/cRTDBInputAdapter.cpp)
target_link_libraries(rtdbInputAdapter tpstores mpClient worldModelInterface)

add_library(rtdbOutputAdapter  src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(rtdbOutputAdapter FalconsTypes)

add_library(rtdbAdapterRefboxSignals src/adapters/cRtdbAdapterRefboxSignals.cpp)
target_link_libraries(rtdbAdapterRefboxSignals FalconsTypes)

add_library(tpHeightmapVisualizer src/heightmaps/heightmapVisualizer.cpp)
target_link_libraries(tpHeightmapVisualizer tpstores rtdbInputAdapter tpconfigAdapter opencv_imgproc)

## TESTERS
# WorldStateFunctions
#add_executable(cWorldStateFunctionsTest tst/cWorldStateFunctionsTest.cpp)
#target_link_libraries(cWorldStateFunctionsTest worldModelInterface tpworldStateFunctions gtest)
#add_test(cWorldStateFunctionsTest cWorldStateFunctionsTest)
#set_tests_properties(cWorldStateFunctionsTest PROPERTIES TIMEOUT 5)
#
## GameState Tester
#add_executable(gameStateTest tst/gameStateTest.cpp)
#target_link_libraries(gameStateTest tpgameStateManager gtest)
#add_test(gameStateTest gameStateTest)
#set_tests_properties(gameStateTest PROPERTIES TIMEOUT 5)
#
## GameStateTransitionTable Tester
#add_executable(gameStateTransitionTableTest tst/gameStateTransitionTableTest.cpp)
#target_link_libraries(gameStateTransitionTableTest tpgameStateManager gtest)
#add_test(gameStateTransitionTableTest gameStateTransitionTableTest)
#set_tests_properties(gameStateTransitionTableTest PROPERTIES TIMEOUT 5)
#
## AbstractHeightMap Tester
#add_executable(abstractHeightMapTest tst/abstractHeightMapTest.cpp)
#target_link_libraries(abstractHeightMapTest tpstores gtest)
#add_test(abstractHeightMapTest abstractHeightMapTest)
#set_tests_properties(abstractHeightMapTest PROPERTIES TIMEOUT 1)
#
## HeightMapStore Tester
#add_executable(heightMapStoreTest tst/heightMapStoreTest.cpp)
#target_link_libraries(heightMapStoreTest tpstores gtest)
#add_test(heightMapStoreTest heightMapStoreTest)
#set_tests_properties(heightMapStoreTest PROPERTIES TIMEOUT 1)
#
## HeightmapVisualizer Tester
#add_executable(heightmapVisualizerTest tst/heightmapVisualizerTest.cpp)
#target_link_libraries(heightmapVisualizerTest tpHeightmapVisualizer opencv_imgproc opencv_highgui opencv_core opencv_imgcodecs gtest)
#add_test(heightmapVisualizerTest heightmapVisualizerTest)
#set_tests_properties(heightmapVisualizerTest PROPERTIES TIMEOUT 10)
#
## HeightMap hmBetweenPoiAndClosestObstacle Tester
#add_executable(hmBetweenPoiAndClosestObstacleTest tst/hmBetweenPoiAndClosestObstacleTest.cpp)
#target_link_libraries(hmBetweenPoiAndClosestObstacleTest tpstores gtest)
#add_test(hmBetweenPoiAndClosestObstacleTest hmBetweenPoiAndClosestObstacleTest)
#set_tests_properties(hmBetweenPoiAndClosestObstacleTest PROPERTIES TIMEOUT 1)
#
## HeightMap hmInFrontOfOppGoal Tester
#add_executable(hmInFrontOfOppGoalTest tst/hmInFrontOfOppGoalTest.cpp)
#target_link_libraries(hmInFrontOfOppGoalTest tpstores gtest)
#add_test(hmInFrontOfOppGoalTest hmInFrontOfOppGoalTest)
#set_tests_properties(hmInFrontOfOppGoalTest PROPERTIES TIMEOUT 1)
#
## HeightMap hmNearPosition Tester
#add_executable(hmNearPositionTest tst/hmNearPositionTest.cpp)
#target_link_libraries(hmNearPositionTest tpstores gtest)
#add_test(hmNearPositionTest hmNearPositionTest)
#set_tests_properties(hmNearPositionTest PROPERTIES TIMEOUT 1)
#
## ObjectPath Tester
#add_executable(cObjectPathTest tst/cObjectPathTest.cpp)
#target_link_libraries(cObjectPathTest objectPath gtest)
#add_test(cObjectPathTest cObjectPathTest)
#set_tests_properties(cObjectPathTest PROPERTIES TIMEOUT 1)
#
## ObstacleStore Tester
#add_executable(obstacleStoreTest tst/obstacleStoreTest.cpp)
#target_link_libraries(obstacleStoreTest tpstores gtest)
#add_test(obstacleStoreTest obstacleStoreTest)
#set_tests_properties(obstacleStoreTest PROPERTIES TIMEOUT 1)
#
## RobotStore Tester
#add_executable(robotStoreTest tst/robotStoreTest.cpp)
#target_link_libraries(robotStoreTest tpstores gtest)
#add_test(robotStoreTest robotStoreTest)
#set_tests_properties(robotStoreTest PROPERTIES TIMEOUT 1)
#
## RuleAvoidAreas Tester
#add_executable(ruleAvoidAreasTest tst/ruleAvoidAreasTest.cpp)
#target_link_libraries(ruleAvoidAreasTest gmock tprules tpstores gtest)
#add_test(ruleAvoidAreasTest ruleAvoidAreasTest)
#set_tests_properties(ruleAvoidAreasTest PROPERTIES TIMEOUT 1)
#
## RuleSetPieceExecuteFinished Tester
#add_executable(ruleSetpieceExecuteFinishedTest tst/ruleSetpieceExecuteFinishedTest.cpp)
#target_link_libraries(ruleSetpieceExecuteFinishedTest gmock tprules tpstores gtest)
#add_test(ruleSetpieceExecuteFinishedTest ruleSetpieceExecuteFinishedTest)
#set_tests_properties(ruleSetpieceExecuteFinishedTest PROPERTIES TIMEOUT 1)
#
## RuleStimulatePassing Tester
#add_executable(ruleStimulatePassingTest tst/ruleStimulatePassingTest.cpp)
#target_link_libraries(ruleStimulatePassingTest tprules tpstores gtest)
#add_test(ruleStimulatePassingTest ruleStimulatePassingTest)
#set_tests_properties(ruleStimulatePassingTest PROPERTIES TIMEOUT 1)
#
## Timer tester
#add_executable(timerTest tst/timerTest.cpp src/utilities/timer.cpp)
#target_link_libraries(timerTest ${Boost_LIBRARIES} gtest Tracing RtDButils FTime)
#add_test(timerTest timerTest)
#set_tests_properties(timerTest PROPERTIES TIMEOUT 1)
#
## Tree tester
#add_executable(cTreeTesterTest tst/cTreeTester.cpp)
#target_link_libraries(cTreeTesterTest tpdecisionTree tpworldStateFunctions ${Boost_LIBRARIES} gtest)
#add_test(cTreeTesterTest cTreeTesterTest)
#set_tests_properties(cTreeTesterTest PROPERTIES TIMEOUT 5)


# MAIN 
add_executable(teamplay_main  src/main.cpp)
target_link_libraries(teamplay_main tpconfigAdapter
                                    rtdbInputAdapter
                                    rtdbOutputAdapter
                                    teamplayControlInterface
                                    worldModelInterface 
                                    rtdbAdapterRefboxSignals 
                                    tpgameStateManager 
                                    tpdecisionTree
                                    tpactions
                                    ${Boost_LIBRARIES}
                                    Tracing
                                    FalconsRTDB
                                    diagnostics
                                    EnvironmentField
                                    falconsGeometry
                                    PathPlanning
                                    mpClient)
