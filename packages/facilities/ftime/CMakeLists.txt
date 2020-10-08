cmake_minimum_required(VERSION 3.5.1)

project(FTime)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)

set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")


include_directories("include/ext")

#Create library libFTime
add_library(FTime src/ftime.cpp)
target_link_libraries(FTime
    ${Boost_LIBRARIES}
    rtdb2
    )


