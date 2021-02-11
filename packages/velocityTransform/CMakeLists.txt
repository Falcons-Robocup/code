cmake_minimum_required(VERSION 3.5.1)

project(VelocityTransform)

find_package(Boost COMPONENTS system thread chrono REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("include")

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
                                          rtdb2
                                          FalconsCommon
                                          diagnostics)
