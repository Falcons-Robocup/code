cmake_minimum_required(VERSION 3.5.1)

project(ShootPlanning)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

add_library(spshootAdapters src/adapters/cRTDBInputAdapter.cpp
                            src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(spshootAdapters FalconsTypes)

add_executable(shoot_planner src/main.cpp
                             src/cShotCalibration.cpp
                             src/cShotSolver.cpp
                             src/cShootPlanner.cpp)
target_link_libraries(shoot_planner spshootAdapters yaml-cpp ${Boost_LIBRARIES} Tracing FalconsCommon diagnostics FalconsRTDB filters)


add_executable(dumpTables  tst/dumpTables.cpp
                           src/cShotCalibration.cpp
                           src/cShotSolver.cpp)
target_link_libraries(dumpTables Tracing FalconsRTDB FalconsTypes)


