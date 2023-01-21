cmake_minimum_required(VERSION 3.5.1)

project(ByteArray)

find_package(Facilities)

include_directories("include/ext" "include")

add_library(byteArray src/cByteArray.cpp
                      src/cAbstractObserverByteArray.cpp
                      )

