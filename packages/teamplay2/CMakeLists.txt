cmake_minimum_required(VERSION 3.5.1)

project(Teamplay2)

find_package(Boost COMPONENTS system thread filesystem REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)
find_package(VelocityControl)
find_package(MixedTeamProtocol)

find_package(BehaviorTree)


#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("include")

# Add link library for finding OpenCV libs
link_directories(/usr/local/lib)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

add_compile_options(-std=c++2a)



## tpGameStateManager
add_library(tpGameStateManager   src/gamestate/GameStateManager.cpp
                                 src/gamestate/GameStateTransitionTable.cpp)
target_link_libraries(tpGameStateManager tpTypes tpRules tpTimer tpStores diagnostics yaml-cpp)


## tpStores
add_library(tpStores            src/stores/BallStore.cpp
                                src/stores/ConfigurationStore.cpp
                                src/stores/DiagnosticsStore.cpp
                                src/stores/FieldDimensionsStore.cpp
                                src/stores/GameStateStore.cpp
                                src/stores/HeightMapStore.cpp
                                src/stores/ObstacleStore.cpp
                                src/stores/RobotStore.cpp)
target_link_libraries(tpStores tpTypes tpHeightmaps diagnostics)


## tpTypes
add_library(tpTypes             src/types/Ball.cpp
                                src/types/Configuration.cpp
                                src/types/FieldDimensions.cpp
                                src/types/GameState.cpp
                                src/types/Obstacle.cpp
                                src/types/Robot.cpp
                                src/types/Role.cpp)
target_link_libraries(tpTypes EnvironmentField tpStores)


## tpRules
add_library(tpRules             src/rules/RuleAvoidAreas.cpp
                                src/rules/RuleSetpieceExecuteFinished.cpp
                                src/rules/RuleStimulatePassing.cpp)
target_link_libraries(tpRules)


## tpHeightmaps
add_library(tpHeightmaps        src/heightmaps/AbstractHeightMap.cpp
                                src/heightmaps/hmAvoidBall.cpp
                                src/heightmaps/hmAvoidObstacles.cpp
                                src/heightmaps/hmAvoidTeamMates.cpp
                                src/heightmaps/hmBetweenPoiAndClosestObstacle.cpp
                                src/heightmaps/hmCloseToBallClaimedLocation.cpp
                                src/heightmaps/hmCloseToOwnPos.cpp
                                src/heightmaps/hmInFrontOfOppGoal.cpp
                                src/heightmaps/hmNearObstacles.cpp
                                src/heightmaps/hmNearOwnGoal.cpp
                                src/heightmaps/hmObstaclesBlockingTeammates.cpp
                                src/heightmaps/hmObstaclesBlockingBall.cpp
                                src/heightmaps/hmObstaclesBlockingOppGoal.cpp)
target_link_libraries(tpHeightmaps tpStores falconsGeometry opencv_core)


## tpHeightmapVisualizer
add_library(tpHeightmapVisualizer src/heightmaps/HeightmapVisualizer.cpp)
target_link_libraries(tpHeightmapVisualizer tpStores tpRtdbAdapters opencv_imgproc)


## tpControlNodes
add_library(tpControlNodes      src/controlnodes/SimpleQueries.cpp
                                src/controlnodes/AdvancedQueries.cpp
                                src/controlnodes/SequenceStarAsyncNode.cpp
                                src/WorldStateFunctions.cpp)
target_link_libraries(tpControlNodes tpStores tpRules tpRtdbAdapters tpTimer behaviortree_cpp_v3)
                                     

## tpDecoratorNodes
add_library(tpDecoratorNodes      src/decoratornodes/AvoidPOI.cpp)
target_link_libraries(tpDecoratorNodes tpStores behaviortree_cpp_v3)


## tpTimer
add_library(tpTimer  src/utilities/Timer.cpp)
target_link_libraries(tpTimer)



## tpRtdbAdapters
add_library(tpRtdbAdapters  src/adapters/RTDBDiagnosticsAdapter.cpp
                            src/adapters/RTDBInputAdapter.cpp
                            src/adapters/RTDBOutputAdapter.cpp
                            src/adapters/RTDBAdapterRefboxSignals.cpp)
target_link_libraries(tpRtdbAdapters tpRules tpGameStateManager tpStores tpTypes mpClient FalconsTypes mixedTeamProtocol MixedTeamProtocolAdapter)


# MAIN
add_executable(teamplay_main2   src/main.cpp

                                src/actionnodes/AbstractAction.cpp
                                src/actionnodes/GetBall.cpp
                                src/actionnodes/GoalKeeper.cpp
                                src/actionnodes/InterceptBall.cpp
                                src/actionnodes/Move.cpp
                                src/actionnodes/PositionBeforePOI.cpp
                                src/actionnodes/PositionBehindPOI.cpp
                                src/actionnodes/PositionWithHeightmap.cpp
                                src/actionnodes/SetRole.cpp
                                src/actionnodes/ShootToPOI.cpp
                                src/actionnodes/TurnAwayFromOpponent.cpp

                                src/plays/PlaySelection.cpp
                                src/plays/RoleAssignment.cpp
                                )


target_link_libraries(teamplay_main2 tpRtdbAdapters
                                     tpStores
                                     tpTypes
                                     tpRules
                                     tpHeightmaps
                                     tpControlNodes
                                     tpDecoratorNodes
                                     tpTimer
                                     tpGameStateManager
                                     ${Boost_LIBRARIES}
                                     mixedTeamProtocol
                                     MixedTeamProtocolAdapter
                                     Tracing
                                     diagnostics
                                     behaviortree_cpp_v3)



#############
## TESTERS ##
#############

# WorldStateFunctions
add_executable(WorldStateFunctionsTest tst/WorldStateFunctionsTest.cpp src/WorldStateFunctions.cpp)
target_link_libraries(WorldStateFunctionsTest tpStores gtest)
add_test(WorldStateFunctionsTest WorldStateFunctionsTest)
set_tests_properties(WorldStateFunctionsTest PROPERTIES TIMEOUT 5)


### gamestate
# GameState Tester
add_executable(GameStateTest tst/gamestate/GameStateTest.cpp)
target_link_libraries(GameStateTest tpGameStateManager gtest pthread)
add_test(GameStateTest GameStateTest)
set_tests_properties(GameStateTest PROPERTIES TIMEOUT 5)

# GameStateTransitionTable Tester
add_executable(GameStateTransitionTableTest tst/gamestate/GameStateTransitionTableTest.cpp)
target_link_libraries(GameStateTransitionTableTest tpGameStateManager gtest pthread)
add_test(GameStateTransitionTableTest GameStateTransitionTableTest)
set_tests_properties(GameStateTransitionTableTest PROPERTIES TIMEOUT 5)


### heightmaps
# Thailand, Bangkok, RoboCup 2022
# TODO: Could not get these tests to work in a short enough timespan
# AbstractHeightMap Tester
#add_executable(AbstractHeightMapTest tst/heightmaps/AbstractHeightMapTest.cpp)
#target_link_libraries(AbstractHeightMapTest tpHeightmaps gtest)
#add_test(AbstractHeightMapTest AbstractHeightMapTest)
#set_tests_properties(AbstractHeightMapTest PROPERTIES TIMEOUT 1)

# HeightMapStore Tester
add_executable(HeightMapStoreTest tst/heightmaps/HeightMapStoreTest.cpp)
target_link_libraries(HeightMapStoreTest tpHeightmaps gtest)
add_test(HeightMapStoreTest HeightMapStoreTest)
set_tests_properties(HeightMapStoreTest PROPERTIES TIMEOUT 1)

# HeightmapVisualizer Tester
add_executable(HeightmapVisualizerTest tst/heightmaps/HeightmapVisualizerTest.cpp)
target_link_libraries(HeightmapVisualizerTest tpHeightmapVisualizer opencv_imgproc opencv_highgui opencv_core opencv_imgcodecs gtest)
add_test(HeightmapVisualizerTest HeightmapVisualizerTest)
set_tests_properties(HeightmapVisualizerTest PROPERTIES TIMEOUT 10)

# HeightMap hmBetweenPoiAndClosestObstacle Tester
add_executable(hmBetweenPoiAndClosestObstacleTest tst/heightmaps/hmBetweenPoiAndClosestObstacleTest.cpp)
target_link_libraries(hmBetweenPoiAndClosestObstacleTest tpHeightmaps gtest pthread)
add_test(hmBetweenPoiAndClosestObstacleTest hmBetweenPoiAndClosestObstacleTest)
set_tests_properties(hmBetweenPoiAndClosestObstacleTest PROPERTIES TIMEOUT 1)

# HeightMap hmInFrontOfOppGoal Tester
add_executable(hmInFrontOfOppGoalTest tst/heightmaps/hmInFrontOfOppGoalTest.cpp)
target_link_libraries(hmInFrontOfOppGoalTest tpHeightmaps gtest pthread)
add_test(hmInFrontOfOppGoalTest hmInFrontOfOppGoalTest)
set_tests_properties(hmInFrontOfOppGoalTest PROPERTIES TIMEOUT 1)


### stores
# ObstacleStore Tester
add_executable(ObstacleStoreTest tst/stores/ObstacleStoreTest.cpp)
target_link_libraries(ObstacleStoreTest tpStores gtest)
add_test(ObstacleStoreTest ObstacleStoreTest)
set_tests_properties(ObstacleStoreTest PROPERTIES TIMEOUT 1)

# RobotStore Tester
add_executable(RobotStoreTest tst/stores/RobotStoreTest.cpp)
target_link_libraries(RobotStoreTest tpStores gtest)
add_test(RobotStoreTest RobotStoreTest)
set_tests_properties(RobotStoreTest PROPERTIES TIMEOUT 1)


### rules
# RuleAvoidAreas Tester
add_executable(RuleAvoidAreasTest tst/rules/RuleAvoidAreasTest.cpp)
target_link_libraries(RuleAvoidAreasTest gmock tpRules tpStores tpTimer gtest)
add_test(RuleAvoidAreasTest RuleAvoidAreasTest)
set_tests_properties(RuleAvoidAreasTest PROPERTIES TIMEOUT 1)

# RuleSetPieceExecuteFinished Tester
add_executable(RuleSetpieceExecuteFinishedTest tst/rules/RuleSetpieceExecuteFinishedTest.cpp)
target_link_libraries(RuleSetpieceExecuteFinishedTest gmock tpRules tpStores tpTimer gtest)
add_test(RuleSetpieceExecuteFinishedTest RuleSetpieceExecuteFinishedTest)
set_tests_properties(RuleSetpieceExecuteFinishedTest PROPERTIES TIMEOUT 1)

# RuleStimulatePassing Tester
add_executable(RuleStimulatePassingTest tst/rules/RuleStimulatePassingTest.cpp)
target_link_libraries(RuleStimulatePassingTest tpRules tpStores gtest)
add_test(RuleStimulatePassingTest RuleStimulatePassingTest)
set_tests_properties(RuleStimulatePassingTest PROPERTIES TIMEOUT 1)


### controlnodes
# SimpleQueries Tester
add_executable(SimpleQueriesTest tst/controlnodes/SimpleQueriesTest.cpp)
target_link_libraries(SimpleQueriesTest tpControlNodes gtest pthread)
add_test(SimpleQueriesTest SimpleQueriesTest)
set_tests_properties(SimpleQueriesTest PROPERTIES TIMEOUT 1)

# AdvancedQueries Tester
add_executable(AdvancedQueriesTest tst/controlnodes/AdvancedQueriesTest.cpp)
target_link_libraries(AdvancedQueriesTest tpControlNodes gtest pthread)
add_test(AdvancedQueriesTest AdvancedQueriesTest)
set_tests_properties(AdvancedQueriesTest PROPERTIES TIMEOUT 1)


### utilities
# Timer tester
add_executable(TimerTest tst/utilities/TimerTest.cpp src/utilities/Timer.cpp)
target_link_libraries(TimerTest ${Boost_LIBRARIES} gtest Tracing RtDButils FTime)
add_test(TimerTest TimerTest)
set_tests_properties(TimerTest PROPERTIES TIMEOUT 1)


