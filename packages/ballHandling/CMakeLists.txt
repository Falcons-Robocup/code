cmake_minimum_required(VERSION 3.5.1)

project(BallHandling)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

# libraries
add_library(bhController  src/ballHandlingControl.cpp)
target_link_libraries(bhController FalconsTypes)

add_library(bhRtdbAdapters  src/adapters/cRTDBInputAdapter.cpp
                            src/adapters/cRTDBOutputAdapter.cpp)
target_link_libraries(bhRtdbAdapters FalconsTypes)

# main
add_executable(ballHandling src/main.cpp)
target_link_libraries(ballHandling bhController bhRtdbAdapters Tracing FalconsRTDB diagnostics ${Boost_LIBRARIES})

