cmake_minimum_required(VERSION 3.5.1)

project(Logging)

find_package(Facilities)
find_package(Boost COMPONENTS system thread REQUIRED QUIET)

include_directories("inc" $ENV{FALCONS_DATA_PATH}/external/)

add_library(logging src/cLogger.cpp src/cLogFilePlayback.cpp src/cLogPlayback.cpp src/cDbConnection.cpp src/cDbSync.cpp src/cLogFileWriter.cpp src/cLogFileReader.cpp src/cFrameBuffer.cpp)
target_link_libraries(logging FalconsRTDB FTime Tracing ${Boost_LIBRARIES})

add_executable(logger src/mLogger.cpp)
target_link_libraries(logger logging FalconsCommon)

add_executable(rdlinfo src/mLogInfo.cpp)
target_link_libraries(rdlinfo logging FalconsCommon)

add_executable(rdldump src/mLogDump.cpp)
target_link_libraries(rdldump logging FalconsCommon)

add_executable(rtdbSync src/mRtdbSync.cpp)
target_link_libraries(rtdbSync logging FalconsCommon)

add_test(testRdl python3 $ENV{FALCONS_CODE_PATH}/packages/facilities/logging/tst/testRdl.py)
set_tests_properties(testRdl PROPERTIES TIMEOUT 10)

