cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)
#set(ROS_BUILD_TYPE Debug)
set(ROS_BUILD_TYPE Release)

rosbuild_init()
rosbuild_add_boost_directories()

# TKOV: enable enumerate value to be scoped
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

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

rosbuild_add_library(wmClient src/adapters/cWorldModelClient.cpp
                              src/adapters/RTDB/RTDBConfigAdapter.cpp)

rosbuild_add_library(wmobjectType src/types/object/objectResultType.cpp)

rosbuild_add_library(wmballTypes src/types/ball/ballType.cpp)
target_link_libraries(wmballTypes wmobjectType)

rosbuild_add_library(wmobstacleTypes src/types/obstacle/obstacleType.cpp)

rosbuild_add_library(wmrobotTypes src/types/robot/robotMeasurementType.cpp
                                src/types/robot/robotType.cpp
                                src/types/robot/robotDisplacementType.cpp
                                src/types/robot/robotVelocityType.cpp
                                )

rosbuild_add_library(wmobjectTracking src/algorithms/objectMeasurementCache.cpp
                                    src/algorithms/objectCoreFit.cpp
                                    )
target_link_libraries(wmobjectTracking wmobjectType)

rosbuild_add_library(wmballTracking   src/administrators/ballDiscriminator.cpp
                                    src/administrators/ballTracker.cpp
                                    src/algorithms/objectTracking.cpp
                                    )
target_link_libraries(wmballTracking opencv_core)
target_link_libraries(wmballTracking wmconfigurators)
target_link_libraries(wmballTracking wmfacilities)
target_link_libraries(wmballTracking wmballTypes)
target_link_libraries(wmballTracking wmobjectTracking)

rosbuild_add_library(wmobstacleTracking src/administrators/obstacleDiscriminator.cpp
                                        src/administrators/obstacleTracker.cpp
                                        src/algorithms/objectTracking.cpp
                                        )
target_link_libraries(wmobstacleTracking opencv_core)
target_link_libraries(wmobstacleTracking wmconfigurators)
target_link_libraries(wmobstacleTracking wmfacilities)
target_link_libraries(wmobstacleTracking wmobstacleTypes)
target_link_libraries(wmobstacleTracking wmobjectTracking)
 
rosbuild_add_library(wmadministrators src/administrators/robotAdministrator.cpp
                                    src/administrators/ballAdministrator.cpp
                                    src/administrators/obstacleAdministrator.cpp
                                    src/administrators/obstacleDiscriminator.cpp
                                    )
target_link_libraries(wmadministrators wmballTracking)
target_link_libraries(wmadministrators wmobstacleTracking)
target_link_libraries(wmadministrators wmlocalization)

rosbuild_add_library(wmconfigurators src/configurators/administrationConfigurator.cpp
                                   src/configurators/localizationConfigurator.cpp
                                   src/configurators/ballTrackerConfigurator.cpp
                                   src/configurators/obstacleTrackerConfigurator.cpp
                                   )
                                                                                        
rosbuild_add_library(wmfacilities src/facilities/identifierGenerator.cpp
                                )

rosbuild_add_library(wmlocalization src/algorithms/robotLocalization.cpp
                                  src/algorithms/localizationTracker.cpp
                                  )

target_link_libraries(wmlocalization wmconfigurators)
target_link_libraries(wmlocalization wmfacilities wmballTypes) # TODO: unexpected link dependency to ballTypes
                             
rosbuild_add_library(wmadapters src/adapters/adaptersCollector.cpp
                              src/adapters/RTDB/RTDBInputAdapter.cpp
                              src/adapters/RTDB/RTDBOutputAdapter.cpp
                              src/adapters/RTDB/RTDBHeartBeatAdapter.cpp
                              src/adapters/RTDB/RTDBConfigAdapter.cpp
                              src/adapters/ROS/heartBeatAdapterROS.cpp
                              src/adapters/configurators/administratorConfigROS.cpp
                              src/adapters/configurators/ballTrackerConfigROS.cpp
                              src/adapters/configurators/localizationConfigROS.cpp
                              src/adapters/configurators/obstacleTrackerConfigROS.cpp
                              )
target_link_libraries(wmadapters wmfacilities)
target_link_libraries(wmadapters wmballTypes)
target_link_libraries(wmadapters wmrobotTypes)
target_link_libraries(wmadapters wmobstacleTypes)
target_link_libraries(wmadapters wmadministrators)

rosbuild_add_library(libWorldModel src/cWorldModel.cpp)
target_link_libraries(libWorldModel wmballTypes)
target_link_libraries(libWorldModel wmobstacleTypes)
target_link_libraries(libWorldModel wmrobotTypes)
target_link_libraries(libWorldModel wmballTracking)
target_link_libraries(libWorldModel wmobstacleTracking)
target_link_libraries(libWorldModel wmadministrators)
target_link_libraries(libWorldModel wmconfigurators)
target_link_libraries(libWorldModel wmfacilities)
target_link_libraries(libWorldModel wmlocalization)
target_link_libraries(libWorldModel wmadapters)

rosbuild_add_executable(worldModelNode src/mWorldModel.cpp)
target_link_libraries(worldModelNode libWorldModel)
rosbuild_link_boost(worldModelNode system thread)

############### TESTERS ####################
rosbuild_add_executable(worldModelStimulator tst/stimulator/mStimulator.cpp tst/stimulator/cWorldModelStimulator.cpp tst/stimulator/cAbstractStimulator.cpp)
target_link_libraries(worldModelStimulator libWorldModel)
rosbuild_link_boost(worldModelStimulator program_options system thread)

rosbuild_add_executable(testObstacleDiscriminator tst/obstacles/testObstacleDiscriminator.cpp)
target_link_libraries(testObstacleDiscriminator wmobstacleTypes wmadministrators wmrobotTypes)
rosbuild_link_boost(testObstacleDiscriminator system thread)

rosbuild_add_pyunit(tst/testWmRobot.py TIMEOUT 200.0)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})
