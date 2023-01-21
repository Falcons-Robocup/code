cmake_minimum_required(VERSION 3.5.1)

project(execution)

find_package(Boost COMPONENTS system thread chrono REQUIRED QUIET)
find_package(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

                                       
add_executable(heartBeatCoach src/mainHeartBeatCoach.cpp)
target_link_libraries(heartBeatCoach
    FalconsCommon
    FalconsRTDB
    Tracing
    diagnostics
    ${Boost_LIBRARIES}
)

add_executable(mainMatchMode src/mainMatchMode.cpp)
target_link_libraries(mainMatchMode
    FalconsCommon
    FalconsRTDB
)

add_executable(execution src/mainExecution.cpp)
target_link_libraries(execution
    FalconsCommon
    FalconsRTDB
    Tracing
    diagnostics
    ${Boost_LIBRARIES}
)
