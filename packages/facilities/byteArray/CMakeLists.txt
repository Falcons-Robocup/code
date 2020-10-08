cmake_minimum_required(VERSION 3.5.1)

project(ByteArray)

find_package(Facilities)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")

include_directories("include/ext" "include")

add_library(byteArray src/cByteArray.cpp
                      src/cAbstractObserverByteArray.cpp
                      )

