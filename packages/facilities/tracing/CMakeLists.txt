cmake_minimum_required(VERSION 3.5.1)

project(Tracing)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)

add_compile_options(-fPIC)

include_directories("include/ext")

#Create library libTracing
add_library(Tracing src/tracing.cpp src/minitrace.c)
target_link_libraries(Tracing
    ${Boost_LIBRARIES}
    )


