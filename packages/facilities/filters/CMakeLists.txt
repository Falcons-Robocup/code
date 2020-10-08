cmake_minimum_required(VERSION 3.5.1)

find_package(Boost COMPONENTS program_options REQUIRED QUIET)


set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x -Wall -Werror")
include_directories("inc")

add_library(filters
    src/AbstractInterpolator.cpp
    src/LeastSquaresInterpolator.cpp
    src/LinearInterpolator.cpp)
target_link_libraries(filters alglib)

add_executable(applyFilter tst/applyFilter.cpp)
target_link_libraries(applyFilter filters ${Boost_LIBRARIES})

# Test
add_executable(filterTest tst/filterTest.cpp)
target_link_libraries(filterTest filters gtest pthread)
add_test(filterTest filterTest)
set_tests_properties(filterTest PROPERTIES TIMEOUT 5)

