cmake_minimum_required(VERSION 3.5.1)

project(FalconsRTDB)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities) # sharedTypes and RtDB

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

add_compile_options(-fPIC)

include_directories("include/ext")

add_library(FalconsRTDB src/FalconsRTDB.cpp)
target_link_libraries(FalconsRTDB
    ${Boost_LIBRARIES}
    RtDBrtdb
    )

