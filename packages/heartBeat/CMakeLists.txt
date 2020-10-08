cmake_minimum_required(VERSION 3.5.1)

project(heartBeat)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

                                       
add_executable(heartBeatCoach src/mainHeartBeatCoach.cpp)
target_link_libraries(heartBeatCoach
    FalconsCommon
    rtdb2
    Tracing
    diagnostics
    ${Boost_LIBRARIES}
)

add_executable(mainMatchMode src/mainMatchMode.cpp)
target_link_libraries(mainMatchMode
    FalconsCommon
    rtdb2
)
