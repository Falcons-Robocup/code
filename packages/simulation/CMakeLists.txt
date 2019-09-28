cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
set(ROS_BUILD_TYPE Debug)

rosbuild_init()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#Add dynamic reconfigure api
#rosbuild_find_ros_package(dynamic_reconfigure)
#include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
#gencfg()

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

include_directories("include/int")

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
rosbuild_add_library(
    arbiter
    src/arbiter.cpp
)

target_link_libraries(
    arbiter
    arbiterControllers
    arbiterGameData
    refboxAdapter
)

rosbuild_add_library(
    arbiterControllers
    src/preparingController.cpp
    src/runningController.cpp
    src/stoppedController.cpp
    src/stoppingController.cpp
)

target_link_libraries(
    arbiterControllers
    arbiterGameData
)

rosbuild_add_library(
    arbiterGameData
    src/arbiterGameData.cpp
)

target_link_libraries(
    arbiterGameData
    gameData
)

rosbuild_add_library(
    gameData
    src/ball.cpp
    src/gameData.cpp
    src/gameDataFactory.cpp
    src/robot.cpp
    src/teamID.cpp
)

rosbuild_add_library(
    configAdapter
    src/RTDBconfigAdapter.cpp
)

target_link_libraries(
    configAdapter
    RTDBaccess
)

rosbuild_add_library(
    gameDataAdapter
    src/RTDBgameDataAdapter.cpp
)

target_link_libraries(
    gameDataAdapter
    RTDBaccess
)

rosbuild_add_library(
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

rosbuild_add_library(
    refboxAdapter
    src/RTDBrefboxAdapter.cpp
)

target_link_libraries(
    refboxAdapter
    RTDBaccess
)

add_dependencies(
    refboxAdapter
    enum_generator
)

rosbuild_add_library(
    RTDBaccess
    src/RTDBaccess.cpp
)

rosbuild_add_library(
    simworld
    src/simworld.cpp
)

target_link_libraries(
    simworld
    arbiter
    configAdapter
    gameDataAdapter
    motionAdapter
    simworldGameDataFactory
)

rosbuild_add_library(
    simworldGameData
    src/simworldGameData.cpp
)

target_link_libraries(
    simworldGameData
    gameData
)

rosbuild_add_library(
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
rosbuild_add_executable(
    sw
    src/simworld_main.cpp
)

target_link_libraries(
    sw
    simworld
)

#############################
# Testers
#############################
rosbuild_add_gtest(
    arbiterTest
    tst/arbiterTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    arbiterTest
    arbiter
    gmock
)

rosbuild_add_gtest(
    ballTest
    tst/ballTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    ballTest
    gameData
)

rosbuild_add_gtest(
    gameDataTest
    tst/gameDataTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    gameDataTest
    gameData
)

rosbuild_add_gtest(
    robotTest
    tst/robotTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    robotTest
    gameData
)

rosbuild_add_gtest(
    preparingControllerTest
    tst/preparingControllerTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    preparingControllerTest
    arbiterControllers
    gmock
)

rosbuild_add_gtest(
    runningControllerTest
    tst/runningControllerTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    runningControllerTest
    arbiterControllers
    gmock
)

rosbuild_add_gtest(
    simworldTest
    tst/simworldTest.cpp
    TIMEOUT 10  # The timeout is longer, because simworld sleeps by design
)

target_link_libraries(
    simworldTest
    simworld
    gmock
)

rosbuild_add_gtest(
    simworldGameDataTest
    tst/simworldGameDataTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    simworldGameDataTest
    simworldGameData
)

rosbuild_add_gtest(
    simworldGameDataFactoryTest
    tst/simworldGameDataFactoryTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    simworldGameDataFactoryTest
    simworldGameDataFactory
)

rosbuild_add_gtest(
    stoppedControllerTest
    tst/stoppedControllerTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    stoppedControllerTest
    arbiterControllers
    gmock
)

rosbuild_add_gtest(
    stoppingControllerTest
    tst/stoppingControllerTest.cpp
    TIMEOUT 1
)

target_link_libraries(
    stoppingControllerTest
    arbiterControllers
)
