cmake_minimum_required(VERSION 3.5.1)

project(RaspiControl)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)
#find_package(Facilities)
find_package(MultiCam)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include_directories("include")
include_directories(/usr/include/opencv4)

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)



# one executable: raspiControl (not production)
# one library: raspiControlLib, used in raspiSync which runs as part of standard robot software
add_library(raspiControlLib src/raspiControl.cpp)
target_link_libraries(raspiControlLib mcRaspiCore)

add_executable(raspiControl src/raspiControlMain.cpp src/raspiControl.cpp)
target_link_libraries(raspiControl raspiControlLib)
target_link_libraries(raspiControl opencv_core)
target_link_libraries(raspiControl opencv_highgui)
target_link_libraries(raspiControl pthread)
target_link_libraries(raspiControl mcRaspiCore)

