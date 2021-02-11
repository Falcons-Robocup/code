cmake_minimum_required(VERSION 3.5.1)

project(PathPlanning)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

include_directories("include")

add_library(ppRtdbAdapters src/adapters/RTDBInputAdapter.cpp
                           src/adapters/RTDBOutputAdapter.cpp)
target_link_libraries(ppRtdbAdapters FalconsCommon)

add_library(PathPlanning src/PathPlanning.cpp
                         src/PathPlanningData.cpp
                         src/PathPlanningClient.cpp
                         src/facilities/ppgeometry.cpp
                         src/algorithms/RequireWorldModelActive.cpp
                         src/algorithms/CheckStopCommand.cpp
                         src/algorithms/CheckTargetValid.cpp
                         src/algorithms/CheckTargetReached.cpp
                         src/algorithms/Shielding.cpp
                         src/algorithms/ForwardDriving.cpp
                         src/algorithms/CalculateObstacles.cpp
                         src/algorithms/EscapeForbiddenAreas.cpp
                         src/algorithms/AvoidObstacles.cpp)
target_link_libraries(PathPlanning ppRtdbAdapters wmClient Tracing FalconsCommon rtdb2 diagnostics EnvironmentField ${Boost_LIBRARIES})

add_executable(pathplanningNode src/main.cpp)
target_link_libraries(pathplanningNode PathPlanning ${Boost_LIBRARIES})

add_executable(pathPlanningTest tst/pathPlanningTest.cpp tst/pathPlanningTestDefaults.cpp)
target_link_libraries(pathPlanningTest PathPlanning yaml-cpp ${Boost_LIBRARIES} gtest logging)
add_test(pathPlanningTest pathPlanningTest)
set_tests_properties(pathPlanningTest PROPERTIES TIMEOUT 60)

