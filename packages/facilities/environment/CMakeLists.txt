cmake_minimum_required(VERSION 3.5.1)

project(Environment)

find_package(Facilities)

include_directories("include/ext" "include")


#Create library libEnvironmentCommon
add_library(EnvironmentCommon src/cEnvironmentCommon.cpp)
target_link_libraries(EnvironmentCommon yaml-cpp FalconsCommon)

#Create library libEnvironmentField
add_library(EnvironmentField src/cEnvironmentField.cpp)
target_link_libraries(EnvironmentField EnvironmentCommon Tracing FalconsRTDB FalconsCommon)

#Create library libEnvironmentBall
add_library(EnvironmentBall src/cEnvironmentBall.cpp)
target_link_libraries(EnvironmentBall EnvironmentCommon)

#Create library libEnvironmentRobot
add_library(EnvironmentRobot src/cEnvironmentRobot.cpp)
target_link_libraries(EnvironmentRobot EnvironmentCommon)

#Create library libEnvironment
add_library(Environment src/cEnvironmentCommon.cpp src/cEnvironmentField.cpp src/cEnvironmentBall.cpp src/cEnvironmentRobot.cpp)

target_link_libraries(Environment yaml-cpp FalconsCommon)

#Testers
add_executable( cEnvironmentTest tst/cEnvironmentTest.cpp)
target_link_libraries( cEnvironmentTest EnvironmentCommon EnvironmentField gtest )
add_test(cEnvironmentTest cEnvironmentTest)
set_tests_properties(cEnvironmentTest PROPERTIES TIMEOUT 1)


add_subdirectory(pymodule)

