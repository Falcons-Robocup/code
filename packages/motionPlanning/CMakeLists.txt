cmake_minimum_required(VERSION 3.5.1)

project(MotionPlanning)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(PathPlanning)
find_package(WorldModel)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

add_library(mpClient src/adapters/cMotionPlanningClient.cpp
                     src/cMotionPlanner.cpp)
target_link_libraries(mpClient wmClient PathPlanning)

add_executable(motionPlanning   src/main.cpp
                                src/cMotionPlanner.cpp)
target_link_libraries(motionPlanning mpInterfaces mpAdapters mpTypes mpActions mpConfigAdapter wmClient PathPlanning ${Boost_LIBRARIES})

add_library(mpAdapters    src/adapters/MP_RTDBInputAdapter.cpp
                          src/adapters/MP_RTDBOutputAdapter.cpp)
target_link_libraries(mpAdapters FalconsCommon)

add_library(mpConfigAdapter  src/adapters/configuration/cConfigMotionPlanningData.cpp)
target_link_libraries(mpConfigAdapter FalconsCommon)

add_library(mpInterfaces  src/cPathPlanningInterface.cpp
                          src/cBallHandlingInterface.cpp
                          src/cQueryInterface.cpp)
target_link_libraries(mpInterfaces Tracing ${Boost_LIBRARIES} wmClient)

add_library(mpTypes       src/types/cForbiddenAreaType.cpp)

add_library(mpActions     src/MP_AbstractAction.cpp
                          src/cTimer.cpp
                          src/algorithms/intercept.cpp
                          src/MP_ActionKick.cpp
                          src/MP_ActionShootAtTarget.cpp
                          src/MP_ActionPassToTarget.cpp
                          src/MP_ActionMoveToTarget.cpp
                          src/MP_ActionGetBall.cpp
                          src/MP_ActionInterceptBall.cpp
                          src/MP_ActionStop.cpp
                          src/MP_ActionTurnAwayFromOpponent.cpp
                          src/MP_ActionKeeperMove.cpp)
target_link_libraries(mpActions falconsGeometry FalconsCommon)


target_link_libraries(mpInterfaces mpTypes)
target_link_libraries(mpAdapters mpTypes)
target_link_libraries(mpClient mpInterfaces)
target_link_libraries(mpClient mpAdapters yaml-cpp)
target_link_libraries(mpClient mpTypes)
target_link_libraries(mpClient mpActions)
target_link_libraries(mpClient mpConfigAdapter)
target_link_libraries(mpClient ${Boost_LIBRARIES})


# TESTERS
# cQueryInterface -- disabled for now, see ticket #122
#add_executable(cQueryInterfaceTest tst/cQueryInterfaceTest.cpp)
#target_link_libraries(cQueryInterfaceTest mpInterfaces gtest)
#add_test(cQueryInterfaceTest cQueryInterfaceTest)
#set_tests_properties(cQueryInterfaceTest PROPERTIES TIMEOUT 1)


