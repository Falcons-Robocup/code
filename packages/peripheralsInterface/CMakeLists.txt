cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()

################################################################################
# Generic libraries
################################################################################
rosbuild_add_library(serial src/Serial.cpp)

################################################################################
# Motors Node
################################################################################
rosbuild_add_library(data src/PeripheralsInterfaceData.cpp)

rosbuild_add_library(motorCommunication src/Communication.cpp
                                        src/CommunicationPackage.cpp)
target_link_libraries(motorCommunication serial)

rosbuild_add_library(deviceManager src/DeviceManager.cpp)
target_link_libraries(deviceManager motorCommunication)

rosbuild_add_library(motor src/motors/MotorControllerBoard.cpp
                           src/motors/BallhandlerBoard.cpp
                           src/motors/MotionBoard.cpp)
target_link_libraries(motor motorCommunication
                            deviceManager)

rosbuild_add_library(motion src/Motion.cpp)
target_link_libraries(motion data
                             motor)

rosbuild_add_library(ballhandlers src/Ballhandlers.cpp)
target_link_libraries(ballhandlers data
                                   motor)

rosbuild_add_library(piRtdbAdapters src/adapters/cRTDBInputAdapter.cpp
                                    src/adapters/cRTDBOutputAdapter.cpp)

rosbuild_add_library(rosAdaptersMotors src/adapters/cRosAdapterMotionConfig.cpp
                                       src/adapters/cRosAdapterBallhandlersConfig.cpp)
target_link_libraries(data)

rosbuild_add_executable(motors src/main_motors.cpp
                               src/Diagnostics.cpp
                               src/VoltageMonitor.cpp)
target_link_libraries(motors deviceManager
                             motion
                             ballhandlers
                             rosAdaptersMotors
                             piRtdbAdapters)
rosbuild_link_boost(motors system thread)

################################################################################
# IoBoard Node
################################################################################
rosbuild_add_library(ioBoardCommunication src/ioBoard/IoBoard.cpp
                                          src/ioBoard/IoBoardCommunication.cpp)
target_link_libraries(ioBoardCommunication serial)

rosbuild_add_library(rosAdaptersIoBoard src/adapters/cRosAdapterKicker.cpp)

rosbuild_add_library(rtdbAdaptersIoBoard src/adapters/cRTDBInputKickerAdapter.cpp
                                         src/adapters/RTDBInputKeeperFrameAdapter.cpp
                                         src/adapters/cRTDBAdapterRobotStatus.cpp)

rosbuild_add_executable(ioBoard src/main_ioBoard.cpp
                                src/KeeperFrame.cpp
                                src/Kicker.cpp)
target_link_libraries(ioBoard ioBoardCommunication
                              rosAdaptersIoBoard
                              rtdbAdaptersIoBoard)
rosbuild_link_boost(ioBoard system thread)

rosbuild_add_executable(testKeeperFrame src/testKeeperFrame.cpp src/Kicker.cpp)
target_link_libraries(testKeeperFrame ioBoardCommunication)
rosbuild_link_boost(testKeeperFrame system thread)

################################################################################
# Tests
################################################################################
rosbuild_add_gtest(serialTest tst/SerialTest.cpp TIMEOUT 1)
target_link_libraries(serialTest serial util)

rosbuild_add_library(ioBoardLib src/ioBoard/IoBoard.cpp)
target_link_libraries(ioBoardLib ioBoardCommunication
                                 rosAdaptersIoBoard)
                              
rosbuild_add_gtest(ioBoardTest tst/IoBoardTest.cpp TIMEOUT 1)
target_link_libraries(ioBoardTest ioBoardLib serial util)
