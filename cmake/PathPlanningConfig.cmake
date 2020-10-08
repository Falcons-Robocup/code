cmake_minimum_required(VERSION 3.5.1)

include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/pathPlanning/include/ext)

# Reflexxes Type II Motion Library
include_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/include)
link_directories($ENV{FALCONS_DATA_PATH}/external/ReflexxesTypeII/lib)

