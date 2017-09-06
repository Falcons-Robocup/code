cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()
rosbuild_add_boost_directories()

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11")


#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)



set(SRCS
    src/adapters/cInputUdpAdapter.cpp
    src/adapters/cProcessAdapter.cpp
    src/adapters/cCommandAdapter.cpp
    src/adapters/cConfigAdapter.cpp
    src/adapters/cDiagnosticsAdapter.cpp
    src/cRobotControl.cpp
    src/cRobotHealth.cpp
    src/cRobotControlProcesses.cpp
    src/cRobotControlConfig.cpp
    src/cRobotControlClaiming.cpp
)

# TODO: consider moving robotHealth to a dedicated package. It is completely separated with robotControl, has much less dependencies.
rosbuild_add_executable(robotControlDaemon src/mainRobotControl.cpp ${SRCS})
rosbuild_add_executable(robotHealth src/mainRobotHealth.cpp ${SRCS})
rosbuild_link_boost(robotControlDaemon program_options)
