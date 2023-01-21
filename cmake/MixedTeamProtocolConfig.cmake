cmake_minimum_required(VERSION 3.5.1)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/mixedTeamProtocol/mtp/include/ext)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/mixedTeamProtocol/adapter/include/ext)

# MunkRes Library
include_directories($ENV{FALCONS_DATA_PATH}/external/munkres-cpp/src)
link_directories($ENV{FALCONS_DATA_PATH}/external/munkres-cpp/build)
