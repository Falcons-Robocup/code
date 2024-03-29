cmake_minimum_required(VERSION 3.5.1)

project(WorldModel)

find_package(Boost COMPONENTS system thread program_options REQUIRED QUIET)
find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")


add_library(wmClient src/adapters/cWorldModelClient.cpp
                     src/adapters/RTDB/RTDBConfigAdapter.cpp)
target_link_libraries(wmClient FalconsCommon FalconsRTDB diagnostics)

add_library(wmobjectType src/types/object/objectResultType.cpp)

add_library(wmballTypes src/types/ball/ballType.cpp)
target_link_libraries(wmballTypes wmobjectType)

add_library(wmobstacleTypes src/types/obstacle/obstacleType.cpp)

add_library(wmrobotTypes src/types/robot/robotMeasurementType.cpp
                         src/types/robot/robotType.cpp
                         src/types/robot/robotDisplacementType.cpp
                         src/types/robot/robotVelocityType.cpp
                         )

add_library(wmobjectTracking src/algorithms/objectMeasurementCache.cpp
                             src/algorithms/objectCoreFit.cpp
                             )
target_link_libraries(wmobjectTracking wmobjectType FalconsTypes)

add_library(wmballTracking   src/administrators/ballDiscriminator.cpp
                             src/administrators/gaussian3DBallDiscriminator.cpp
                             src/administrators/gaussian3DPosVelObject.cpp
                             src/administrators/ballTracker.cpp
                             src/algorithms/objectTracking.cpp
                             )
target_link_libraries(wmballTracking opencv_core)
target_link_libraries(wmballTracking wmfacilities)
target_link_libraries(wmballTracking wmballTypes)
target_link_libraries(wmballTracking wmobjectTracking)
target_link_libraries(wmballTracking EnvironmentField)
target_link_libraries(wmballTracking filters)

add_library(wmobstacleTracking src/administrators/obstacleDiscriminator.cpp
                               src/administrators/obstacleTracker.cpp
                               src/algorithms/objectTracking.cpp
                               )
target_link_libraries(wmobstacleTracking opencv_core)
target_link_libraries(wmobstacleTracking wmfacilities)
target_link_libraries(wmobstacleTracking wmobstacleTypes)
target_link_libraries(wmobstacleTracking wmobjectTracking)
 
add_library(wmadministrators src/administrators/robotAdministrator.cpp
                             src/administrators/ballAdministrator.cpp
                             src/administrators/obstacleAdministrator.cpp
                             src/administrators/obstacleDiscriminator.cpp
                             )
target_link_libraries(wmadministrators wmballTracking)
target_link_libraries(wmadministrators wmobstacleTracking)
target_link_libraries(wmadministrators wmlocalization)
target_link_libraries(wmadministrators wmrobotTypes)
target_link_libraries(wmadministrators falconsGeometry)
target_link_libraries(wmadministrators diagnostics)

add_library(wmfacilities src/facilities/identifierGenerator.cpp)

add_library(wmlocalization src/algorithms/robotLocalization.cpp
                           src/algorithms/localizationTracker.cpp
                           )

target_link_libraries(wmlocalization wmfacilities wmballTypes) # TODO: unexpected link dependency to ballTypes
                             
add_library(wmadapters src/adapters/adaptersCollector.cpp
                       src/adapters/RTDB/RTDBInputAdapter.cpp
                       src/adapters/RTDB/RTDBOutputAdapter.cpp
                       src/adapters/RTDB/RobotHeartBeatAdapter.cpp
                       src/adapters/RTDB/CoachHeartBeatAdapter.cpp
                       src/adapters/RTDB/RTDBConfigAdapter.cpp
                       src/adapters/configurators/WorldModelConfig.cpp
                       )
target_link_libraries(wmadapters wmfacilities)
target_link_libraries(wmadapters FalconsCommon)
target_link_libraries(wmadapters FalconsRTDB)

add_library(libWorldModel src/cWorldModel.cpp)
target_link_libraries(libWorldModel wmballTypes)
target_link_libraries(libWorldModel wmobstacleTypes)
target_link_libraries(libWorldModel wmrobotTypes)
target_link_libraries(libWorldModel wmballTracking)
target_link_libraries(libWorldModel wmobstacleTracking)
target_link_libraries(libWorldModel wmadministrators)
target_link_libraries(libWorldModel wmfacilities)
target_link_libraries(libWorldModel wmlocalization)
target_link_libraries(libWorldModel wmadapters)

add_executable(worldModelNode src/mWorldModel.cpp)
target_link_libraries(worldModelNode libWorldModel ${Boost_LIBRARIES} Tracing diagnostics)

############### TEST HELPERS ###############

add_executable(worldModelStimulator tst/stimulator/mStimulator.cpp tst/stimulator/cWorldModelStimulator.cpp tst/stimulator/cAbstractStimulator.cpp)
target_link_libraries(worldModelStimulator libWorldModel ${Boost_LIBRARIES} logging)
# worldModelStimulator is not a test by itself - it is used as helper utility in testWmRDLsuite


add_executable(trueBallStimulator tst/stimulator/mTrueBallStimulator.cpp tst/stimulator/cTrueBallStimulator.cpp tst/stimulator/cAbstractStimulator.cpp)
target_link_libraries(trueBallStimulator libWorldModel ${Boost_LIBRARIES} logging opencv_core opencv_highgui opencv_imgproc opencv_videoio)

############### TESTERS ####################

#add_test(testWmRDLsuite python3 $ENV{FALCONS_CODE_PATH}/packages/worldModel/tst/testWmRDLsuite.py -v)
#set_tests_properties(testWmRDLsuite PROPERTIES TIMEOUT 200)
# disabled - test got broken somehow; WorldModel needs a new maintainer.

