cmake_minimum_required(VERSION 3.5.1)

project(VelocityTransform)

find_package(Boost COMPONENTS system thread chrono REQUIRED QUIET)
find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

# VelocityTransformClient
add_library(vtClient src/cVelocityTransformClient.cpp)

# RTDB adapters
add_library(vtRtdbAdapters src/adapters/cRTDBInputAdapter.cpp
                           src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(vtRtdbAdapters FalconsTypes)

add_library(velocityTransformAlgorithms src/cAbstractVelocityTransform.cpp
                                        src/algorithms/cTranslateFeedback.cpp
                                        src/algorithms/cPublishFeedback.cpp
                                        src/algorithms/cTranslateTarget.cpp
                                        src/algorithms/cPublishTarget.cpp)
target_link_libraries(velocityTransformAlgorithms FalconsTypes)

# main
add_executable(velocityTransform src/main.cpp
                                 src/cVelocityTransformMain.cpp
                                 src/cVelocityTransformData.cpp)

# linking
target_link_libraries(velocityTransform   vtRtdbAdapters
                                          velocityTransformAlgorithms
                                          ${Boost_LIBRARIES}
                                          Tracing
                                          FalconsRTDB
                                          FalconsCommon
                                          diagnostics)
