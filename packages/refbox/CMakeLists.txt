cmake_minimum_required(VERSION 3.5.1)

project(Refbox)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(WorldModel)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)


add_library(refboxFeedback src/logger/cPacketRefboxLogger.cpp)
target_link_libraries(refboxFeedback json-c)

add_executable(refboxRelay src/main.cpp src/cRefboxRelay.cpp src/RTDBAdapterLogging.cpp src/TCPIP_client.cpp)
target_link_libraries(refboxRelay refboxFeedback ${Boost_LIBRARIES} wmClient Tracing rtdb2 diagnostics)


#############################
# Testers
##############################
add_executable(cPacketRefBoxLoggerTests tst/cPacketRefBoxLoggerTests.cpp)
target_link_libraries(cPacketRefBoxLoggerTests refboxFeedback gtest pthread)
add_test(cPacketRefBoxLoggerTests cPacketRefBoxLoggerTests)
set_tests_properties(cPacketRefBoxLoggerTests PROPERTIES TIMEOUT 10)
