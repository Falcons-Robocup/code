#++
#
# $Id: CMakeLists.txt 2014 2015-07-16 08:53:44Z tkov $
#
#--
cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
set(ROS_BUILD_TYPE Debug)
#set(ROS_BUILD_TYPE Release)

rosbuild_init()

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
#rosbuild_genmsg()
#uncomment if you have defined services
#rosbuild_gensrv()

#add dynamic reconfigure api
rosbuild_find_ros_package(dynamic_reconfigure)
include(${dynamic_reconfigure_PACKAGE_PATH}/cmake/cfgbuild.cmake)
gencfg()



rosbuild_add_library(adaptersWmSync src/adapters/configuratorWorldModelPacketROS.cpp
                              src/adapters/configuratorMixedTeamPacketROS.cpp
                              src/adapters/wmInfoPacketROS.cpp
                              src/adapters/wmInfoUDPPacketROS.cpp
                              src/adapters/wmInfoUDPPacketROScoach.cpp
                              src/adapters/mixedTeamInfoPacketROS.cpp
                              )                          

rosbuild_add_library(packetsWmSync src/constructors/packetConstructorWorldModel.cpp
                                   src/constructors/packetConstructorMixedTeam.cpp
                                   src/cPacketLossAdministrator.cpp
                             )
                             
rosbuild_add_library(configuratorsWmSync src/configurators/configuratorWorldModelPacket.cpp
                                         src/configurators/configuratorMixedTeamPacket.cpp                             
                                         )
                                   
rosbuild_add_library(transceiversWmSync src/transceivers/transmitterWorldModel.cpp
                                  src/transceivers/receiverWorldModel.cpp
                                  src/transceivers/receiverMixedTeam.cpp
                                  src/transceivers/transmitterMixedTeam.cpp
                                  )  
target_link_libraries(transceiversWmSync configuratorsWmSync) 
target_link_libraries(transceiversWmSync packetsWmSync)     

# robot send+receive 
rosbuild_add_executable(worldModelSyncNode src/mainROS.cpp)                                                        
target_link_libraries(worldModelSyncNode adaptersWmSync) 
target_link_libraries(worldModelSyncNode configuratorsWmSync) 
target_link_libraries(worldModelSyncNode transceiversWmSync) 
target_link_libraries(worldModelSyncNode packetsWmSync) 

# coach listener, for visualization and diagnostics
rosbuild_add_executable(worldModelSyncNodeCoach src/mainROScoach.cpp) 
target_link_libraries(worldModelSyncNodeCoach adaptersWmSync) 
target_link_libraries(worldModelSyncNodeCoach configuratorsWmSync) 
target_link_libraries(worldModelSyncNodeCoach transceiversWmSync) 
target_link_libraries(worldModelSyncNodeCoach packetsWmSync) 

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
