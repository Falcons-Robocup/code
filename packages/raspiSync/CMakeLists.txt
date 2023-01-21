cmake_minimum_required(VERSION 3.5.1)

project(RaspiSync)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(MultiCam)
find_package(RaspiControl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

include_directories("include")


# Create executable for raspiSync
add_executable(raspiSync src/camGrabReceive.cpp src/raspiSync.cpp)
target_link_libraries(raspiSync opencv_core)
target_link_libraries(raspiSync opencv_highgui)
target_link_libraries(raspiSync opencv_imgproc)
target_link_libraries(raspiSync pthread)
target_link_libraries(raspiSync raspiControlLib)
