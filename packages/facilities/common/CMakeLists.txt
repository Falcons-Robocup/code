cmake_minimum_required(VERSION 3.5.1)

find_package(Boost COMPONENTS system thread filesystem REQUIRED QUIET)
find_package(Facilities)


add_compile_options(-fPIC)

include_directories("include/ext" "include")

# TODO cleanup this component a bit - there is quite some unused stuff, or stuff that belongs in dedicated packages

#Create library libFalconsCommon
add_library(FalconsCommon
    src/falconsCommonDirs.cpp
    src/falconsCommonEnv.cpp
    src/falconsCommonConfig.cpp
    src/falconsCommonLegacy.cpp
    src/cLinearRegression.cpp
)

target_link_libraries(FalconsCommon
                      FalconsTypes
                      Tracing
                      ${Boost_LIBRARIES})

# Add dependency on generated_enum2str.hpp created by sharedTypes
add_dependencies(FalconsCommon FalconsTypes)

#Testers
add_executable(testPose2D tst/testPose2D.cpp)
target_link_libraries(testPose2D gtest pthread)
add_test(testPose2D testPose2D)
set_tests_properties(testPose2D PROPERTIES TIMEOUT 1)

add_executable(testVelocity2D tst/testVelocity2D.cpp)
target_link_libraries(testVelocity2D gtest pthread)
add_test(testVelocity2D testVelocity2D)
set_tests_properties(testVelocity2D PROPERTIES TIMEOUT 1)

add_executable(testMatrix33 tst/testMatrix33.cpp)
target_link_libraries(testMatrix33 gtest pthread opencv_core)
add_test(testMatrix33 testMatrix33)
set_tests_properties(testMatrix33 PROPERTIES TIMEOUT 1)
