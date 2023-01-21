cmake_minimum_required(VERSION 3.5.1)

project(mtpAdapter)

# Falcons adapter around MTP

find_package(MixedTeamProtocol)
find_package(RtDB)
find_package(Facilities) # need only falconsCommonEnv

find_package(Boost COMPONENTS program_options REQUIRED QUIET)

include_directories("include")

add_library(MixedTeamProtocolAdapter
    src/MTPAdapter.cpp
    src/MTPAdapterImpl.cpp)
target_link_libraries(MixedTeamProtocolAdapter mixedTeamProtocol)

add_executable(mtpAdapter
    src/main.cpp)
target_link_libraries(mtpAdapter FalconsCommon FalconsRTDB diagnostics)

add_executable(MTPExternalHumanMain
    src/MTPExternalHumanVision.cpp
    src/MTPExternalHumanMain.cpp)
target_link_libraries(MTPExternalHumanMain mixedTeamProtocol ${Boost_LIBRARIES})
