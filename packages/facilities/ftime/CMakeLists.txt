cmake_minimum_required(VERSION 3.5.1)

project(FTime)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

include_directories("include/ext")

#Create library libFTime
add_library(FTime src/ftime.cpp)
target_link_libraries(FTime
    ${Boost_LIBRARIES}
    FalconsRTDB
    )


