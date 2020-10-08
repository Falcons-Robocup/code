cmake_minimum_required(VERSION 3.5.1)

project(NetworkUDP)

find_package(Facilities)

set(CMAKE_CXX_FLAGS ${CMAKE_CXX_FLAGS} " -std=c++11 -Wall -Werror")

include_directories("include/ext" "include")

add_library(Network src/cTransmitterUDP.cpp
                    src/cReceiverUDP.cpp
                    src/portsAddresses.cpp
                    ) 

target_link_libraries(Network byteArray)

