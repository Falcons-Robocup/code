cmake_minimum_required(VERSION 3.5.1)

# sharedTypes
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/sharedTypes/include)

# RtDB
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/rtdb/rtdb2)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/rtdb/client/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/rtdb/utils/include)
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/rtdb/definitions/include)

# Falcons RTDB adapter
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/frtdb/include/ext)

# FTime
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/ftime/include/ext)

# common
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/common/include/ext)

# tracing
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/tracing/include/ext)

# filters
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/filters/inc/ext)

# diagnostics
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/diagnostics/include/ext)

# configuration
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/configuration/inc)

# json
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/json/include/ext)

# byteArray
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/byteArray/include/ext)

# geometry
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/geometry/include/ext)
include_directories(/usr/include/opencv4)

# environment
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/environment/include/ext)

# logging
include_directories(${CMAKE_CURRENT_LIST_DIR}/../packages/facilities/logging/inc/ext)

# gmock
include_directories(/usr/src/gmock/include)
