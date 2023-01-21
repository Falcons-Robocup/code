cmake_minimum_required(VERSION 3.5.1)

project(Visualizer)

find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(MixedTeamProtocol)
find_package(Facilities)
find_package(Teamplay2)
find_package(PathPlanning)
find_package(VelocityControl)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -rdynamic")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -rdynamic")

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

# Graphics dependencies
find_package(VTK REQUIRED)
find_package(Qt5Widgets REQUIRED)
#include(${QT_USE_FILE})
include(${VTK_USE_FILE})

# Instruct CMake to run moc automatically when needed.
#set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("include")

# UserInterface files
set(UI
    ui/MainWindow.ui
)

set(MOC
    include/int/MainWindow.h
    include/int/GameSignalSubscriber.h
    include/int/PlaybackControl.h
    include/int/adapters/GameSignalAdapter.h
    include/int/adapters/RtdbGameSignalAdapter.h
    include/int/widgets/Field/FieldWidget3D.h
    include/int/widgets/Field/FieldVideoActor.h
    include/int/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.h
    include/int/widgets/Field/Annotation/Annotation.h
    include/int/widgets/Field/Annotation/PlannedPath.h
    include/int/widgets/Field/Visualization/Visualization.h
    include/int/widgets/Field/Visualization/BallVisualization.h
    include/int/widgets/Field/Visualization/ObstacleVisualization.h
    include/int/widgets/Field/Visualization/GaussianVisualization.h
    include/int/widgets/Field/Visualization/ForbiddenAreaVisualization.h
    include/int/widgets/Field/Visualization/TechnicalTeamAreaVisualization.h
    include/int/widgets/Field/Visualization/RobotVisualization.h
    include/int/widgets/Table/TableWidget.h
    include/int/widgets/Table/TableViewModel.h
    include/int/widgets/GameTimeClock/GameTimeClock.h
    include/int/widgets/Playback/PlaybackWidget.h
    include/int/widgets/EventLogger/EventLogger.h
    include/int/widgets/MatchState/MatchState.h
    include/int/widgets/RobotStatus/BatteryStatus.h
)

qt5_wrap_ui(UI_GENERATED ${UI})
qt5_wrap_cpp(MOC_GENERATED ${MOC})

# Source files
set(SRCS
    src/MainWindow.cpp
    src/PlaybackControl.cpp
    src/TeamRobotSelection.cpp
    src/adapters/RtdbGameSignalAdapter.cpp
    src/widgets/Field/FieldWidget3D.cpp
    src/widgets/Field/FieldWidgetGameSignalSubscriber.cpp
    src/widgets/Field/CamFeedParams.cpp
    src/widgets/Field/FieldVideoActor.cpp
    src/widgets/Field/Annotation/Annotation.cpp
    src/widgets/Field/Annotation/CollisionBalloon.cpp
    src/widgets/Field/Annotation/RobotLabel.cpp
    src/widgets/Field/Annotation/PlannedPath.cpp
    src/widgets/Field/Visualization/Visualization.cpp
    src/widgets/Field/Visualization/BallVisualization.cpp
    src/widgets/Field/Visualization/ObstacleVisualization.cpp
    src/widgets/Field/Visualization/GaussianVisualization.cpp
    src/widgets/Field/Visualization/ForbiddenAreaVisualization.cpp
    src/widgets/Field/Visualization/TechnicalTeamAreaVisualization.cpp
    src/widgets/Field/Visualization/RobotVisualization.cpp
    src/widgets/Field/Visualization/ShootTargetVisualization.cpp
    src/widgets/Field/Visualization/ProjectSpeedVisualization.cpp
    src/widgets/Field/EventHandlers/FieldMouseHoverEventHandler.cpp
    src/widgets/Table/TableViewModel.cpp
    src/widgets/Table/TableWidget.cpp
    src/widgets/Table/TableViewModelGameSignalSubscriber.cpp
    src/widgets/GameTimeClock/GameTimeClock.cpp
    src/widgets/GameTimeClock/GameTimeClockGameSignalSubscriber.cpp
    src/widgets/Playback/PlaybackWidget.cpp
    src/widgets/Playback/PlaybackGameSignalSubscriber.cpp
    src/widgets/EventLogger/EventLogger.cpp
    src/widgets/EventLogger/EventLoggerGameSignalSubscriber.cpp
    src/widgets/MatchState/MatchState.cpp
    src/widgets/MatchState/MatchStateGameSignalSubscriber.cpp
    src/widgets/RobotStatus/BatteryStatus.cpp
    src/widgets/RobotStatus/BatteryStatusGameSignalSubscriber.cpp
)

# Binaries
add_library(
    visualizerLib
    ${SRCS}
    ${MOC_GENERATED}
    ${UI_GENERATED}
)

target_link_libraries(
    visualizerLib
    ${QT_LIBRARIES}
    ${VTK_LIBRARIES}
    glut
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_videoio
    falconsGeometry
    FalconsTypes
    FalconsCommon
)

set_source_files_properties(qrc_breeze.cpp PROPERTIES GENERATED ON)

add_executable(
    visualizer
    src/main.cpp
    ${SRCS}
    ${MOC_GENERATED}
    ${UI_GENERATED}
    rc/breeze.qrc
)

target_link_libraries(
    visualizer
    ${QT_LIBRARIES}
    ${VTK_LIBRARIES}
    glut
    opencv_core
    opencv_highgui
    opencv_imgproc
    opencv_videoio
    Tracing
    FalconsRTDB
    EnvironmentField
    falconsGeometry
    logging
    tpHeightmapVisualizer
    FalconsCommon
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


### Tests

# TableViewModel.h
add_executable(TableViewModelTests tst/TableViewModelTests.cpp)
target_link_libraries(TableViewModelTests visualizerLib gtest)
add_test(TableViewModelTests TableViewModelTests)
set_tests_properties(TableViewModelTests PROPERTIES TIMEOUT 10)

# TeamRobotSelection.h
add_executable(TeamRobotSelectionTests tst/TeamRobotSelectionTests.cpp)
target_link_libraries(TeamRobotSelectionTests visualizerLib gtest)
add_test(TeamRobotSelectionTests TeamRobotSelectionTests)
set_tests_properties(TeamRobotSelectionTests PROPERTIES TIMEOUT 10)

# Visualization.h
add_executable(VisualizationTests tst/VisualizationTests.cpp tst/AnnotationStub.cpp ${MOC_TST_GENERATED2})
target_link_libraries(VisualizationTests visualizerLib gtest pthread)
add_test(VisualizationTests VisualizationTests)
set_tests_properties(VisualizationTests PROPERTIES TIMEOUT 10)

# RobotVisualization.h
add_executable(RobotVisualizationTests tst/RobotVisualizationTests.cpp tst/PlannedPathStub.cpp ${MOC_TST_GENERATED3})
target_link_libraries(RobotVisualizationTests visualizerLib gtest)
add_test(RobotVisualizationTests RobotVisualizationTests)
set_tests_properties(RobotVisualizationTests PROPERTIES TIMEOUT 10)

