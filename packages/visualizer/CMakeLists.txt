cmake_minimum_required(VERSION 2.8.11)

project(visualizer)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
include_directories(include)

# ROS init
set(ROS_BUILD_TYPE Release)
#set(ROS_BUILD_TYPE Debug)

rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

# Need C++11
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "-std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "-std=c++0x")
else()
    message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Graphics dependencies 
find_package(VTK REQUIRED) 
find_package(Qt5Widgets REQUIRED) 
#include(${QT_USE_FILE})
include(${VTK_USE_FILE})

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)

# UserInterface files
set(UI
    ui/MainWindow.ui
    ui/FieldWidgetSettings.ui
)

set(MOC
    include/int/MainWindow.h
    include/int/GameSignalSubscriber.h
    include/int/GameSignalAdapter.h
    include/int/PlaybackControl.h
    include/int/RtdbGameSignalAdapter.h
    include/int/widgets/Field/FieldWidget3D.h
    include/int/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.h
    include/int/widgets/Field/Annotation/Annotation.h
    include/int/widgets/Field/Annotation/PlannedPath.h
    include/int/widgets/Field/Visualization/Visualization.h
    include/int/widgets/Field/Visualization/BallVisualization.h
    include/int/widgets/Field/Visualization/ObstacleVisualization.h
    include/int/widgets/Field/Visualization/GaussianVisualization.h
    include/int/widgets/Field/Visualization/ForbiddenAreaVisualization.h
    include/int/widgets/Field/Visualization/RobotVisualization.h
    include/int/widgets/Table/TableWidget.h
    include/int/widgets/Table/TableViewModel.h
    include/int/widgets/GameTimeClock.h
    include/int/widgets/Playback/PlaybackWidget.h
    include/int/widgets/EventLogger.h
    include/int/widgets/MatchState.h
    include/int/widgets/BatteryStatus.h
)

set(RCC
    rc/visualizer.qrc
    rc/robot.stl
)

qt5_wrap_ui(UI_GENERATED ${UI})
qt5_wrap_cpp(MOC_GENERATED ${MOC})
qt5_add_resources(RC_GENERATED ${RCC})

# Source files
set(SRCS 
    src/RtdbGameSignalAdapter.cpp
    src/RtdbRefboxConfigAdapter.cpp
    src/TeamRobotSelection.cpp
    src/widgets/Field/FieldWidget3D.cpp
    src/widgets/Field/FieldWidgetGameSignalSubscriber.cpp
    src/widgets/Field/CamFeedParams.cpp
    src/widgets/Field/Annotation/Annotation.cpp
    src/widgets/Field/Annotation/CollisionBalloon.cpp
    src/widgets/Field/Annotation/RobotLabel.cpp
    src/widgets/Field/Annotation/PlannedPath.cpp
    src/widgets/Field/Visualization/Visualization.cpp
    src/widgets/Field/Visualization/BallVisualization.cpp
    src/widgets/Field/Visualization/ObstacleVisualization.cpp
    src/widgets/Field/Visualization/GaussianVisualization.cpp
    src/widgets/Field/Visualization/ForbiddenAreaVisualization.cpp
    src/widgets/Field/Visualization/RobotVisualization.cpp
    src/widgets/Field/Visualization/ShootTargetVisualization.cpp
    src/widgets/Field/Visualization/ProjectSpeedVisualization.cpp
    src/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.cpp
    src/widgets/Table/TableViewModel.cpp
    src/widgets/Table/TableWidget.cpp
    src/widgets/Table/TableViewModelGameSignalSubscriber.cpp
    src/widgets/GameTimeClock.cpp
    src/widgets/GameTimeClockGameSignalSubscriber.cpp
    src/widgets/Playback/PlaybackWidget.cpp
    src/widgets/Playback/PlaybackGameSignalSubscriber.cpp
    src/PlaybackControl.cpp
    src/widgets/SettingsDialog.cpp
    src/widgets/EventLogger.cpp
    src/widgets/EventLoggerGameSignalSubscriber.cpp
    src/widgets/MatchState.cpp
    src/widgets/MatchStateGameSignalSubscriber.cpp
    src/widgets/BatteryStatus.cpp
    src/widgets/BatteryStatusGameSignalSubscriber.cpp
    src/MainWindow.cpp
)

# Binaries
rosbuild_add_library(
    visualizerLib
    ${SRCS}
    ${MOC_GENERATED} 
    ${UI_GENERATED} 
    ${RCC_GENERATED} 
)

target_link_libraries(
    visualizerLib
    ${QT_LIBRARIES}
    ${VTK_LIBRARIES}
    glut
)

rosbuild_add_executable(
    visualizer 
    src/main.cpp 
    ${SRCS}
    ${MOC_GENERATED} 
    ${UI_GENERATED} 
    ${RCC_GENERATED} 
)

target_link_libraries(
    visualizer 
    ${QT_LIBRARIES}
    ${VTK_LIBRARIES}
    glut
)

# Testers
set(MOC_TST1
    tst/GameSignalSubscriberStub.h
)

set(MOC_TST2
    tst/AnnotationStub.h
)

set(MOC_TST3
    tst/PlannedPathStub.h
)

qt5_wrap_cpp(MOC_TST_GENERATED1 ${MOC_TST1})
qt5_wrap_cpp(MOC_TST_GENERATED2 ${MOC_TST2})
qt5_wrap_cpp(MOC_TST_GENERATED3 ${MOC_TST3})

#####################################
#  Below tests DON'T need roscore   #
#####################################

# TableViewModel.h
rosbuild_add_gtest(
    TableViewModelTests
    tst/TableViewModelTests.cpp
    TIMEOUT 10
)

target_link_libraries(
    TableViewModelTests
    visualizerLib
)

# TeamRobotSelection.h
rosbuild_add_gtest(
    TeamRobotSelectionTests
    tst/TeamRobotSelectionTests.cpp
    TIMEOUT 10
)

target_link_libraries(
    TeamRobotSelectionTests
    visualizerLib
)

# Visualization.h
rosbuild_add_gtest(
    VisualizationTests
    tst/VisualizationTests.cpp
    tst/AnnotationStub.cpp
    ${MOC_TST_GENERATED2}
    TIMEOUT 10
)

target_link_libraries(
    VisualizationTests
    visualizerLib
)

# RobotVisualization.h
rosbuild_add_gtest(
    RobotVisualizationTests
    tst/RobotVisualizationTests.cpp
    tst/PlannedPathStub.cpp
    ${MOC_TST_GENERATED3}
    TIMEOUT 10
)

target_link_libraries(
    RobotVisualizationTests
    visualizerLib
)

