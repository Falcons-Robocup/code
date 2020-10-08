cmake_minimum_required(VERSION 3.5.1)

project(BallHandling)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)


set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -Wall -Werror")

include_directories("include")

# libraries
add_library(bhController  src/ballHandlingControl.cpp)
target_link_libraries(bhController FalconsTypes)

add_library(bhRtdbAdapters  src/adapters/cRTDBInputAdapter.cpp
                            src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(bhRtdbAdapters FalconsTypes)

# main
add_executable(ballHandling src/main.cpp)
target_link_libraries(ballHandling bhController bhRtdbAdapters Tracing rtdb2 diagnostics ${Boost_LIBRARIES})

