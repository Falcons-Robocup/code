cmake_minimum_required(VERSION 3.5.1)

project(PeripheralsInterface)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")


################################################################################
# Generic libraries
################################################################################
add_library(serial src/Serial.cpp)

################################################################################
# Motors Node
################################################################################
add_library(data src/motors/PeripheralsInterfaceData.cpp)
target_link_libraries(data FalconsTypes)

add_library(motorCommunication src/motors/Communication.cpp
                               src/motors/CommunicationPackage.cpp)
target_link_libraries(motorCommunication serial)

add_library(deviceManager src/motors/DeviceManager.cpp)
target_link_libraries(deviceManager motorCommunication FalconsTypes)

add_library(motor src/motors/MotorControllerBoard.cpp
                  src/motors/BallhandlerBoard.cpp
                  src/motors/MotionBoard.cpp)
target_link_libraries(motor motorCommunication
                            deviceManager)

add_library(motion src/motors/Motion.cpp)
target_link_libraries(motion data
                             motor)

add_library(ballhandlers src/motors/Ballhandlers.cpp)
target_link_libraries(ballhandlers data
                                   motor)

add_library(piRtdbAdapters src/motors/cRTDBInputAdapter.cpp
                           src/motors/cRTDBOutputAdapter.cpp)

target_link_libraries(piRtdbAdapters FalconsTypes)

add_executable(motors src/motors/main_motors.cpp
                      src/motors/Diagnostics.cpp
                      src/motors/VoltageMonitor.cpp)
target_link_libraries(motors deviceManager
                             motion
                             ballhandlers
                             piRtdbAdapters
                             ${Boost_LIBRARIES}
                             Tracing
                             FalconsCommon
                             rtdb2
                             diagnostics)

################################################################################
# IoBoard Node
################################################################################
add_library(ioBoardCommunication src/ioBoard/IoBoard.cpp
                                 src/ioBoard/IoBoardCommunication.cpp)
target_link_libraries(ioBoardCommunication serial FalconsTypes)

add_library(rtdbAdaptersIoBoard src/ioBoard/cRTDBInputKickerAdapter.cpp
                                src/ioBoard/RTDBInputKeeperFrameAdapter.cpp
                                src/ioBoard/cRTDBAdapterRobotStatus.cpp)
target_link_libraries(rtdbAdaptersIoBoard FalconsTypes)

add_executable(ioBoard
    src/ioBoard/main_ioBoard.cpp
    src/ioBoard/KeeperFrame.cpp
    src/ioBoard/Kicker.cpp)
target_link_libraries(ioBoard
    ioBoardCommunication
    rtdbAdaptersIoBoard
    ${Boost_LIBRARIES}
    Tracing
    rtdb2
    FalconsTypes
    diagnostics)

add_executable(testKeeperFrame src/ioBoard/testKeeperFrame.cpp src/ioBoard/Kicker.cpp)
target_link_libraries(testKeeperFrame ioBoardCommunication ${Boost_LIBRARIES} Tracing rtdb2 diagnostics)

################################################################################
# Tests
################################################################################
add_executable(serialTest tst/SerialTest.cpp)
target_link_libraries(serialTest serial util gtest pthread)
add_test(serialTest serialTest)
set_tests_properties(serialTest PROPERTIES TIMEOUT 1)

add_library(ioBoardLib src/ioBoard/IoBoard.cpp)
target_link_libraries(ioBoardLib ioBoardCommunication)
                              
add_executable(ioBoardTest tst/IoBoardTest.cpp)
target_link_libraries(ioBoardTest ioBoardLib serial util gtest pthread)
add_test(ioBoardTest ioBoardTest)
set_tests_properties(ioBoardTest PROPERTIES TIMEOUT 1)
