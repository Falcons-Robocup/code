cmake_minimum_required(VERSION 3.5.1)

project(VelocityControl)

find_package(Boost COMPONENTS system thread chrono REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall -Werror")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("include")

# RTDB adapters
add_library(vcRtdbAdapters src/adapters/cRTDBInputAdapter.cpp
                           src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(vcRtdbAdapters FalconsTypes)

add_library(velocityControlAlgorithms src/cAbstractVelocityControl.cpp
                                      src/algorithms/cTranslateFeedback.cpp
                                      src/algorithms/cPublishFeedback.cpp
                                      src/algorithms/cTranslateTarget.cpp
                                      src/algorithms/cPublishTarget.cpp)
target_link_libraries(velocityControlAlgorithms FalconsTypes)

# main
add_executable(velocityControl src/main.cpp
                               src/cVelocityControlMain.cpp
                               src/cVelocityControlData.cpp)

# linking
target_link_libraries(velocityControl   vcRtdbAdapters
                                        velocityControlAlgorithms
                                        ${Boost_LIBRARIES}
                                        Tracing
                                        rtdb2
                                        FalconsCommon
                                        diagnostics)
