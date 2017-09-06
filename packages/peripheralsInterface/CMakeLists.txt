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

################################################################################
# BEGIN Override the buildin gtest stuff with our gmock/gtest package
# see also /opt/ros/jade/share/catkin/cmake/test/gtest.cmake
################################################################################
rosbuild_find_ros_package(gmock)
set(GTEST_LIBRARIES gmock)
include_directories(BEFORE ${gmock_PACKAGE_PATH}/include)
set(GTEST_INCLUDE_DIR ${gmock_PACKAGE_PATH}/include CACHE PATH rosoverride FORCE)
set(_CATKIN_GTEST_INCLUDE ${gmock_PACKAGE_PATH}/src/gtest/include/gtest/gtest.h CACHE FILEPATH rosoverride FORCE)
set(_CATKIN_GTEST_SRC ${gmock_PACKAGE_PATH}/src/gtest/src/gtest.cc CACHE FILEPATH rosoverride FORCE)
set(gtest_SOURCE_DIR ${gmock_PACKAGE_PATH}/src/gtest/src CACHE STATIC rosoverride FORCE)
set(GTEST_FROM_SOURCE_INCLUDE_DIRS ${gmock_PACKAGE_PATH}/src/gtest/include CACHE INTERNAL rosoverride FORCE)
################################################################################
# END Override the buildin gtest stuff with our gmock/gtest package
################################################################################

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

rosbuild_add_library(rosAdaptersMotors src/adapters/cRosAdapterMotion.cpp
                                       src/adapters/cRosAdapterMotionConfig.cpp
                                       src/adapters/cRosAdapterBallhandlers.cpp
                                       src/adapters/cRosAdapterBallhandlersConfig.cpp
                                       src/adapters/cRosAdapterWorldModel.cpp)
target_link_libraries(data)

rosbuild_add_executable(motors src/main_motors.cpp
                               src/Diagnostics.cpp
                               src/VoltageMonitor.cpp)
target_link_libraries(motors deviceManager
                             motion
                             ballhandlers
                             rosAdaptersMotors)

################################################################################
# Compass Node
################################################################################
#rosbuild_add_library(IMU src/IMU/YoctoModule.cpp
#                         src/IMU/Yocto3DModule.cpp
#                         src/IMU/IMU.ioBoardcpp)
#target_link_libraries(IMU yocto)
#
#rosbuild_add_library(compass src/compass/cCompass.cpp
#                             src/config/cCompassConfig.cpp)
#
#rosbuild_add_library(rosAdaptersCompass src/adapters/cRosAdapterCompass.cpp
#                                        src/adapters/cRosAdapterCompassConfig.cpp)
#target_link_libraries(rosAdaptersCompass compass)
#
#rosbuild_add_library(rosAdaptersIMU src/adapters/cRosAdapterIMU.cpp
#                                    src/adapters/cRosAdapterIMUConfig.cpp)
#
#rosbuild_add_executable(compass src/main_compass.cpp)
#target_link_libraries(compass compass
#                              IMU
#                              rosAdaptersCompass
#                              rosAdaptersIMU)

################################################################################
# IoBoard Node
################################################################################
rosbuild_add_library(ioBoardCommunication src/ioBoard/IoBoard.cpp
                                          src/ioBoard/IoBoardCommunication.cpp)
target_link_libraries(ioBoardCommunication serial)

rosbuild_add_library(rosAdaptersIoBoard src/adapters/cRosAdapterRobotStatus.cpp
                                        src/adapters/cRosAdapterKicker.cpp)

rosbuild_add_executable(ioBoard src/main_ioBoard.cpp
                                src/Kicker.cpp)
target_link_libraries(ioBoard ioBoardCommunication
                              rosAdaptersIoBoard)

################################################################################
# Tests
################################################################################
rosbuild_add_gtest(serialTest tst/SerialTest.cpp TIMEOUT 1)
target_link_libraries(serialTest serial util)

rosbuild_add_executable(ioBoardTest src/test_ioBoard.cpp)
target_link_libraries(ioBoardTest serial)
