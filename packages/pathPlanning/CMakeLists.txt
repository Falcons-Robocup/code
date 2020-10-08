cmake_minimum_required(VERSION 3.5.1)

project(PathPlanning)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

include_directories("include")


# Reflexxes Type II Motion Library
include_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/include)
link_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/lib)

add_library(rtdbAdapters src/adapters/RTDBInputAdapter.cpp
                         src/adapters/RTDBOutputAdapter.cpp)
target_link_libraries(rtdbAdapters FalconsCommon)

add_library(PathPlanning src/PathPlanning.cpp
                         src/PathPlanningData.cpp
                         src/PathPlanningClient.cpp
                         src/facilities/ppgeometry.cpp
                         src/controllers/StopVelocitySetpointController.cpp
                         src/controllers/LinearVelocitySetpointController.cpp
                         src/controllers/PIDVelocitySetpointController.cpp
                         src/controllers/SPGVelocitySetpointController.cpp
                         src/algorithms/RequireWorldModelActive.cpp
                         src/algorithms/CheckStopCommand.cpp
                         src/algorithms/CheckTargetValid.cpp
                         src/algorithms/CheckTargetReached.cpp
                         src/algorithms/Shielding.cpp
                         src/algorithms/ForwardDriving.cpp
                         src/algorithms/CalculateDeltas.cpp
                         src/algorithms/CalculateObstacles.cpp
                         src/algorithms/SelectVelocityController.cpp
                         src/algorithms/CalculateVelocity.cpp
                         src/algorithms/CalculateAccelerating.cpp
                         src/algorithms/Deadzone.cpp
                         src/algorithms/ApplyLimits.cpp
                         src/algorithms/ShiftBallOffset.cpp
                         src/algorithms/UnShiftBallOffset.cpp
                         src/algorithms/EscapeForbiddenAreas.cpp
                         src/algorithms/AvoidObstacles.cpp)
target_link_libraries(PathPlanning rtdbAdapters wmClient ReflexxesTypeII Tracing FalconsCommon rtdb2 diagnostics EnvironmentField ${Boost_LIBRARIES})

add_executable(pathplanningNode src/main.cpp)
target_link_libraries(pathplanningNode PathPlanning ${Boost_LIBRARIES})

add_executable(pathPlanningTest tst/pathPlanningTest.cpp tst/MiniSimulation.cpp tst/pathPlanningTestDefaults.cpp)
target_link_libraries(pathPlanningTest PathPlanning yaml-cpp ${Boost_LIBRARIES} gtest logging)
add_test(pathPlanningTest pathPlanningTest)
set_tests_properties(pathPlanningTest PROPERTIES TIMEOUT 60)

