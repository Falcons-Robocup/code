cmake_minimum_required(VERSION 3.5.1)

project(MCBInterface)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories($ENV{FALCONS_DATA_PATH}/external/protobuf/include)
link_directories($ENV{FALCONS_DATA_PATH}/external/protobuf/lib)
include_directories($ENV{FALCONS_DATA_PATH}/external/zeromq/include)
link_directories($ENV{FALCONS_DATA_PATH}/external/zeromq/lib)

include_directories("include")

add_library(adapters src/RTDBInputAdapter.cpp
                     src/RTDBOutputAdapter.cpp)
target_link_libraries(adapters FalconsTypes)

add_library(mcb src/Rpc.cpp
                src/Kicker.pb.cc
                src/Kicker.rpc.pb.cc
                src/Motors.pb.cc
                src/Motors.rpc.pb.cc)
target_link_libraries(mcb zmq protobuf)

add_executable(mcbInterface src/main.cpp)
target_link_libraries(mcbInterface adapters mcb pthread Tracing diagnostics)
target_link_libraries(mcbInterface FalconsTypes)

