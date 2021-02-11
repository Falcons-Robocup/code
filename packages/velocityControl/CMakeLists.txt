cmake_minimum_required(VERSION 3.5.1)

project(VelocityControl)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

include_directories("include")


# Reflexxes Type II Motion Library
include_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/include)
link_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/lib)

add_library(vcRtdbAdapters src/adapters/vcRTDBInputAdapter.cpp
                           src/adapters/vcRTDBOutputAdapter.cpp
                           src/adapters/vcConfigRTDBAdapter.cpp)
target_link_libraries(vcRtdbAdapters FalconsCommon)

add_library(vcClient src/VelocityControlClient.cpp)
target_link_libraries(vcClient VelocityControl)

add_library(VelocityControl src/VelocityControl.cpp
                            src/VelocityControlData.cpp
                            src/facilities/vcgeometry.cpp
                            src/controllers/StopVelocitySetpointController.cpp
                            src/controllers/LinearVelocitySetpointController.cpp
                            src/controllers/PIDVelocitySetpointController.cpp
                            src/controllers/SPGVelocitySetpointController.cpp
                            src/algorithms/CalculateDeltas.cpp
                            src/algorithms/SelectVelocityController.cpp
                            src/algorithms/CalculateVelocity.cpp
                            src/algorithms/CalculateAccelerating.cpp
                            src/algorithms/Deadzone.cpp
                            src/algorithms/ApplyLimits.cpp
                            src/algorithms/ShiftBallOffset.cpp
                            src/algorithms/UnShiftBallOffset.cpp)

target_link_libraries(VelocityControl vcRtdbAdapters wmClient ReflexxesTypeII Tracing FalconsCommon rtdb2 diagnostics EnvironmentField ${Boost_LIBRARIES})

add_executable(velocityControlNode src/main.cpp)
target_link_libraries(velocityControlNode vcClient ${Boost_LIBRARIES})

add_executable(velocityControlTest tst/velocityControlTest.cpp tst/velocityControlTestDefaults.cpp)
target_link_libraries(velocityControlTest VelocityControl ${Boost_LIBRARIES} gtest logging)
add_test(velocityControlTest velocityControlTest)
set_tests_properties(velocityControlTest PROPERTIES TIMEOUT 60)