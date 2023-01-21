cmake_minimum_required(VERSION 3.5.1)

project(mixedTeamProtocol)

find_package(RtDB)
find_package(MixedTeamProtocol)
find_package(Boost COMPONENTS thread program_options REQUIRED QUIET)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")

add_library(mixedTeamProtocol
    src/MixedTeamProtocol.cpp
    src/MixedTeamProtocolImpl.cpp
    src/Communication.cpp
    src/AdapterRTDB.cpp
    src/Referee.cpp
    src/Roles.cpp
    src/RoleAllocation/RoleAllocation.cpp
    src/RoleAllocation/BruteForce.cpp
    src/RoleAllocation/KuhnMunkres.cpp
    src/PlayerId.cpp
    src/Player.cpp)
add_library(MTP::mixedTeamProtocol ALIAS mixedTeamProtocol)
target_link_libraries(mixedTeamProtocol RtDBrtdb RtDBcomm)
target_include_directories(mixedTeamProtocol
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/ext>
)

# test build targets

add_library(testLib tst/MatchSimulation.cpp tst/MatchSimulationChecks.cpp tst/RobotClient.cpp)
target_link_libraries(testLib mixedTeamProtocol)

add_executable(matchTests tst/matchTests.cpp)
target_link_libraries(matchTests testLib gtest ${Boost_LIBRARIES} RtDButils)

add_executable(testRoleAllocation tst/TestRoleAllocation.cpp)
target_link_libraries(testRoleAllocation mixedTeamProtocol ${Boost_LIBRARIES})

add_executable(clientSpoofer tst/ClientSpoofer.cpp)
target_link_libraries(clientSpoofer mixedTeamProtocol ${Boost_LIBRARIES})



# test suite

## EKPC - Disabled after discussion with JFEI. Needs an RtDB database clear between tests. Some tests leave a polluted environment, messing up the next test.
#add_test(matchTests matchTests)
#set_tests_properties(matchTests PROPERTIES TIMEOUT 10)

add_test(TestRoleAllocation python3 ${CMAKE_CURRENT_LIST_DIR}/tst/TestRoleAllocation.py)
set_tests_properties(TestRoleAllocation PROPERTIES TIMEOUT 10)

