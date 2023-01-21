cmake_minimum_required(VERSION 3.5.1)

project(Refbox)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")


add_library(refboxFeedback src/logger/cPacketRefboxLogger.cpp)
target_link_libraries(refboxFeedback json-c)

add_executable(refboxRelay src/main.cpp src/cRefboxRelay.cpp src/RTDBAdapterLogging.cpp src/TCPIP_client.cpp)
target_link_libraries(refboxRelay refboxFeedback ${Boost_LIBRARIES} wmClient Tracing FalconsRTDB diagnostics)


#############################
# Testers
##############################
add_executable(cPacketRefBoxLoggerTests tst/cPacketRefBoxLoggerTests.cpp)
target_link_libraries(cPacketRefBoxLoggerTests refboxFeedback gtest pthread)
add_test(cPacketRefBoxLoggerTests cPacketRefBoxLoggerTests)
set_tests_properties(cPacketRefBoxLoggerTests PROPERTIES TIMEOUT 10)
