cmake_minimum_required(VERSION 3.5.1)

project(ShootPlanning)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")

add_library(spshootAdapters src/adapters/cRTDBInputAdapter.cpp
                            src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(spshootAdapters FalconsTypes)

add_executable(shoot_planner src/main.cpp
                             src/cShotCalibration.cpp
                             src/cShotSolver.cpp
                             src/cShootPlanner.cpp)
target_link_libraries(shoot_planner spshootAdapters yaml-cpp ${Boost_LIBRARIES} Tracing FalconsCommon diagnostics rtdb2 filters)


add_executable(dumpTables  tst/dumpTables.cpp
                           src/cShotCalibration.cpp
                           src/cShotSolver.cpp)
target_link_libraries(dumpTables Tracing rtdb2 FalconsTypes)


