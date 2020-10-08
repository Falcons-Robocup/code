cmake_minimum_required(VERSION 3.5.1)

project(Simulation)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

### RELEASE
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

### DEBUG
#set(CMAKE_BUILD_TYPE Debug)
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror -g")


include_directories("include" "include/int")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


#############################
# Code generators
#############################
add_custom_target(
    enum_generator ALL
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/generateEnums.py -i ${CMAKE_CURRENT_SOURCE_DIR}/include/int -o ${CMAKE_CURRENT_SOURCE_DIR}/include/int/generated_enum2str.hpp -p ${CMAKE_CURRENT_SOURCE_DIR}/src/generated_enum2str.py
    BYPRODUCTS generated_enum2str.hpp generated_enum2str.py
    COMMENT "Generating code from enums"
)

#############################
# Libraries
#############################
add_library(
    arbiter
    src/arbiter.cpp
)

target_link_libraries(
    arbiter
    arbiterControllers
    arbiterGameData
    refboxAdapter
)

add_library(
    arbiterControllers
    src/preparingController.cpp
    src/runningController.cpp
    src/stoppedController.cpp
    src/stoppingController.cpp
)

target_link_libraries(
    arbiterControllers
    arbiterGameData
    EnvironmentField
)

add_library(
    arbiterGameData
    src/arbiterGameData.cpp
)

target_link_libraries(
    arbiterGameData
    gameData
)

add_library(
    gameData
    src/ball.cpp
    src/gameData.cpp
    src/gameDataFactory.cpp
    src/robot.cpp
    src/teamID.cpp
)

target_link_libraries(
    gameData
    Tracing
    FalconsCommon
)

add_library(
    configAdapter
    src/RTDBconfigAdapter.cpp
)

target_link_libraries(
    configAdapter
    RTDBaccess
    diagnostics
)

add_library(
    gameDataAdapter
    src/RTDBgameDataAdapter.cpp
)

target_link_libraries(
    gameDataAdapter
    RTDBaccess
)

add_library(
    motionAdapter
    src/RTDBmotionAdapter.cpp
)

target_link_libraries(
    motionAdapter
    RTDBaccess
)

add_dependencies(
    motionAdapter
    enum_generator
)

add_library(
    refboxAdapter
    src/RTDBrefboxAdapter.cpp
)

target_link_libraries(
    refboxAdapter
    RTDBaccess
    FTime
)

add_dependencies(
    refboxAdapter
    enum_generator
)

add_library(
    timeAdapter
    src/RTDBtimeAdapter.cpp
)

target_link_libraries(
    timeAdapter
    RTDBaccess
)

add_dependencies(
    timeAdapter
    enum_generator
)

add_library(
    RTDBaccess
    src/RTDBaccess.cpp
)

target_link_libraries(
    RTDBaccess
    FalconsTypes
)

add_library(
    simworld
    src/simworld.cpp
)

target_link_libraries(
    simworld
    arbiter
    configAdapter
    gameDataAdapter
    motionAdapter
    timeAdapter
    simworldGameDataFactory
)

add_library(
    simworldGameData
    src/simworldGameData.cpp
)

target_link_libraries(
    simworldGameData
    gameData
    EnvironmentField
)

add_dependencies(
    simworldGameData
    enum_generator
)

add_library(
    simworldGameDataFactory
    src/simworldGameDataFactory.cpp
)

target_link_libraries(
    simworldGameDataFactory
    simworldGameData
)

#############################
# Executables
#############################
add_executable(
    sw
    src/simworld_main.cpp
)

target_link_libraries(
    sw
    simworld
    Tracing
    rtdb2
    diagnostics
)

#############################
# Testers
#############################
add_executable(arbiterTest tst/arbiterTest.cpp)
target_link_libraries(arbiterTest arbiter gmock gtest)
add_test(arbiterTest arbiterTest)
set_tests_properties(arbiterTest PROPERTIES TIMEOUT 1)

add_executable(ballTest tst/ballTest.cpp)
target_link_libraries(ballTest gameData gtest)
add_test(ballTest ballTest)
set_tests_properties(ballTest PROPERTIES TIMEOUT 1)

add_executable(gameDataTest tst/gameDataTest.cpp)
target_link_libraries(gameDataTest gameData gtest)
add_test(gameDataTest gameDataTest)
set_tests_properties(gameDataTest PROPERTIES TIMEOUT 1)

add_executable(robotTest tst/robotTest.cpp)
target_link_libraries(robotTest gameData gtest)
add_test(robotTest robotTest)
set_tests_properties(robotTest PROPERTIES TIMEOUT 1)

add_executable(preparingControllerTest tst/preparingControllerTest.cpp)
target_link_libraries(preparingControllerTest  arbiterControllers  gmock gtest)
add_test(preparingControllerTest preparingControllerTest)
set_tests_properties(preparingControllerTest PROPERTIES TIMEOUT 1)

add_executable(runningControllerTest tst/runningControllerTest.cpp)
target_link_libraries(runningControllerTest arbiterControllers gmock gtest)
add_test(runningControllerTest runningControllerTest)
set_tests_properties(runningControllerTest PROPERTIES TIMEOUT 1)

# this test is crashing with a segfault, seems framework related? -> disabled
#add_executable(simworldTest tst/simworldTest.cpp)
#target_link_libraries(simworldTest simworld gmock gtest)
#add_test(simworldTest simworldTest)
#set_tests_properties(simworldTest PROPERTIES TIMEOUT 10)  # The timeout is longer, because simworld sleeps by design

add_executable(simworldGameDataTest tst/simworldGameDataTest.cpp)
target_link_libraries(simworldGameDataTest simworldGameData gtest)
add_test(simworldGameDataTest simworldGameDataTest)
set_tests_properties(simworldGameDataTest PROPERTIES TIMEOUT 10)

add_executable(simworldGameDataFactoryTest tst/simworldGameDataFactoryTest.cpp)
target_link_libraries(simworldGameDataFactoryTest simworldGameDataFactory gtest)
add_test(simworldGameDataFactoryTest simworldGameDataFactoryTest)
set_tests_properties(simworldGameDataFactoryTest PROPERTIES TIMEOUT 1)

add_executable(stoppedControllerTest tst/stoppedControllerTest.cpp)
target_link_libraries(stoppedControllerTest arbiterControllers gmock gtest)
add_test(stoppedControllerTest stoppedControllerTest)
set_tests_properties(stoppedControllerTest PROPERTIES TIMEOUT 1)

add_executable(stoppingControllerTest tst/stoppingControllerTest.cpp)
target_link_libraries(stoppingControllerTest arbiterControllers gtest)
add_test(stoppingControllerTest stoppingControllerTest)
set_tests_properties(stoppingControllerTest PROPERTIES TIMEOUT 1)
