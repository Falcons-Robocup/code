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

# TKOV: enable enumerate value to be scoped
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")

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

rosbuild_add_library(objectType src/types/object/objectMeasurementType.cpp
                                src/types/object/objectResultType.cpp
                                )

rosbuild_add_library(ballTypes src/types/ball/ballType.cpp
                               src/types/ball/ballPossessionType.cpp
                               )
target_link_libraries(ballTypes objectType)

rosbuild_add_library(obstacleTypes src/types/obstacle/obstacleType.cpp)

rosbuild_add_library(robotTypes src/types/robot/robotMeasurementType.cpp
                                src/types/robot/robotType.cpp
                                src/types/robot/robotDisplacementType.cpp
                                )

rosbuild_add_library(objectTracking src/algorithms/objectMeasurementCache.cpp
                                    src/algorithms/objectCoreFit.cpp
                                    )
target_link_libraries(objectTracking objectType)

rosbuild_add_library(ballTracking   src/administrators/ballDiscriminator.cpp
                                    src/administrators/ballTracker.cpp
                                    src/administrators/ballStimulator.cpp
                                    src/algorithms/objectTracking.cpp
                                    )
target_link_libraries(ballTracking opencv_core)
target_link_libraries(ballTracking configurators)
target_link_libraries(ballTracking facilities)
target_link_libraries(ballTracking ballTypes)
target_link_libraries(ballTracking objectTracking)

rosbuild_add_library(obstacleTracking   src/administrators/obstacleDiscriminator.cpp
                                        src/administrators/obstacleTracker.cpp
                                        src/algorithms/objectTracking.cpp
                                        )
target_link_libraries(obstacleTracking opencv_core)
target_link_libraries(obstacleTracking configurators)
target_link_libraries(obstacleTracking facilities)
target_link_libraries(obstacleTracking obstacleTypes)
target_link_libraries(obstacleTracking objectTracking)
 
rosbuild_add_library(administrators src/administrators/robotAdministrator.cpp
                                    src/administrators/ballAdministrator.cpp
                                    src/administrators/obstacleAdministrator.cpp
                                    src/administrators/obstacleDiscriminator.cpp
                                    )
target_link_libraries(administrators ballTracking)
target_link_libraries(administrators obstacleTracking)
target_link_libraries(administrators localization)

rosbuild_add_library(configurators src/configurators/administrationConfigurator.cpp
                                   src/configurators/localizationConfigurator.cpp
                                   src/configurators/ballTrackerConfigurator.cpp
                                   src/configurators/obstacleTrackerConfigurator.cpp
                                   )
                                                                                        
rosbuild_add_library(facilities src/facilities/identifierGenerator.cpp
                                src/facilities/ROSConvert.cpp
                                )

rosbuild_add_library(localization src/algorithms/robotLocalization.cpp
                                  src/algorithms/localizationTracker.cpp
                                  src/administrators/localizationStimulator.cpp
                                  )

target_link_libraries(localization configurators)
                                                                  
rosbuild_add_library(adapters src/adapters/adaptersCollector.cpp
                              src/adapters/worldModelInfoUpdater.cpp
                              src/adapters/worldModelSyncInfoUpdater.cpp
                              src/adapters/ROS/heartBeatAdapterROS.cpp
                              src/adapters/ROS/visionAdapterBallMeasurementROS.cpp
                              src/adapters/ROS/visionAdapterObstacleMeasurementROS.cpp
                              src/adapters/ROS/visionAdapterRobotMeasurementROS
                              src/adapters/ROS/visionAdapterBallPossessionROS.cpp
                              src/adapters/ROS/peripheralsAdapterDisplacementROS.cpp
                              src/adapters/ROS/peripheralsAdapterBallPossessionROS.cpp
                              src/adapters/ROS/peripheralsAdapterRobotStatusROS.cpp
                              src/adapters/ROS/wmSyncAdapterBallMeasurementROS.cpp
                              src/adapters/ROS/wmSyncAdapterObstacleMeasurementROS.cpp
                              src/adapters/ROS/wmSyncAdapterBallPossessionROS.cpp
                              src/adapters/ROS/wmSyncAdapterTeamMemberROS.cpp
                              src/adapters/ROS/worldModelInfoUpdaterROS.cpp
                              src/adapters/ROS/worldModelSyncInfoUpdaterROS.cpp
                              src/adapters/ROS/overruleRobotAdapterROS.cpp
                              src/adapters/ROS/overruleBallAdapterROS.cpp
                              src/adapters/ROS/overruleObstacleAdapterROS.cpp
                              src/adapters/ROS/overruleStatusAdapterROS.cpp
                              src/adapters/configurators/administratorConfigROS.cpp
                              src/adapters/configurators/ballTrackerConfigROS.cpp
                              src/adapters/configurators/localizationConfigROS.cpp
                              src/adapters/configurators/obstacleTrackerConfigROS.cpp
                              )
target_link_libraries(adapters facilities)
target_link_libraries(adapters ballTypes)
target_link_libraries(adapters robotTypes)
target_link_libraries(adapters obstacleTypes)
target_link_libraries(adapters administrators)

rosbuild_add_executable(worldModelNode src/mainROS.cpp)
target_link_libraries(worldModelNode ballTypes)
target_link_libraries(worldModelNode obstacleTypes)
target_link_libraries(worldModelNode robotTypes)
target_link_libraries(worldModelNode ballTracking)
target_link_libraries(worldModelNode obstacleTracking)
target_link_libraries(worldModelNode administrators)
target_link_libraries(worldModelNode configurators)
target_link_libraries(worldModelNode facilities)
target_link_libraries(worldModelNode localization)
target_link_libraries(worldModelNode adapters)

############### TESTERS ####################
rosbuild_add_gtest(ballPossessionTypeTester tst/types/ball/ballPossessionTypeTester.cpp TIMEOUT 5)
target_link_libraries(ballPossessionTypeTester ballTypes)

# JFEI disabled for now, need more time to solve -> branch wmImprovements
#rosbuild_add_gtest(testRobotLocalization tst/algorithms/testRobotLocalization.cpp TIMEOUT 5)
#target_link_libraries(testRobotLocalization localization robotTypes)

rosbuild_add_gtest(testBallDiscriminator tst/ball/testBallDiscriminator.cpp TIMEOUT 5)
target_link_libraries(testBallDiscriminator administrators ballTypes obstacleTypes robotTypes configurators)

rosbuild_add_gtest(testBallAdministrator tst/ball/testBallAdministrator.cpp TIMEOUT 5)
target_link_libraries(testBallAdministrator administrators ballTypes obstacleTypes robotTypes configurators)

rosbuild_add_gtest(testLocalization tst/localization/testLocalization.cpp TIMEOUT 5)
target_link_libraries(testLocalization localization robotTypes configurators)
target_link_libraries(localization facilities ballTypes) # TODO: unexpected link dependency to ballTypes

rosbuild_add_gtest(testLocalizationIO tst/localization/testLocalizationIO.cpp TIMEOUT 5)
target_link_libraries(testLocalizationIO localization robotTypes configurators)

rosbuild_add_gtest(testCollectorAdapter tst/adapters/testCollectorAdapter.cpp TIMEOUT 5)
target_link_libraries(testCollectorAdapter administrators 
                                           ballTypes
                                           obstacleTypes 
                                           robotTypes 
                                           configurators  
                                           facilities  
                                           adapters                                     
                                           gmock
                                           )


#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
