cmake_minimum_required(VERSION 3.5.1)

project(Tracing)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror -fPIC")
set(CMAKE_C_FLAGS "-Wall -Werror -fPIC")

include_directories("include/ext")

#Create library libTracing
add_library(Tracing src/tracing.cpp src/minitrace.c)
target_link_libraries(Tracing
    ${Boost_LIBRARIES}
    )


