cmake_minimum_required(VERSION 3.5.1)

project(mtrbc)

# system dependencies
find_package(RtDB)
find_package(Boost REQUIRED COMPONENTS program_options thread QUIET)
find_package(Qt5 REQUIRED COMPONENTS Core)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# library
add_library(mtrbclient STATIC
    RefboxCommand.cpp
    RefboxProtocol2020Client.cpp
)

target_link_libraries(mtrbclient
    ${Boost_THREAD_LIBRARY}
    Qt5::Core
)

# binary
add_executable(mtrbc main.cpp)
target_link_libraries(mtrbc
    mtrbclient
    RtDB::rtdb
    RtDB::comm
    ${Boost_PROGRAM_OPTIONS_LIBRARY}
)

add_executable(example-client example.cpp)
target_link_libraries(example-client
    RtDB::rtdb
    RtDB::comm
)
