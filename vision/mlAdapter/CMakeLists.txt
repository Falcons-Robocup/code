cmake_minimum_required(VERSION 3.5.1)

project(mlAdapter)

find_package(Facilities)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -g3 -O3 -m64 -ansi -std=c++11 -Wall -Wextra -Wpedantic -Werror")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

include_directories(./include)
include_directories(./../../packages/facilities/common/include/ext)
include_directories(./../../packages/facilities/sharedTypes/include)
            
add_executable(mlAdapter    src/mlAdapter.cpp)

target_link_libraries(mlAdapter
    pthread
    FalconsCommon
    FalconsRTDB
)
