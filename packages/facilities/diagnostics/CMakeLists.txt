cmake_minimum_required(VERSION 3.5.1)

find_package(PythonLibs REQUIRED)
find_package(Facilities)


find_package(Boost COMPONENTS system thread REQUIRED QUIET)

include_directories("include")

# Create library libDiagnostics
add_library(diagnostics src/cDiagnostics.cpp)
target_link_libraries(diagnostics
    Tracing
    RtDButils
    FalconsCommon
    FalconsRTDB
    FTime
    ${Boost_LIBRARIES}
    )

# Executables
add_executable(robotHealth src/mainRobotHealth.cpp src/cRobotHealth.cpp)
target_link_libraries(robotHealth diagnostics ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

add_executable(sendEvent src/sendEvent.cpp)
target_link_libraries(sendEvent diagnostics ${Boost_SYSTEM_LIBRARY} ${Boost_THREAD_LIBRARY})

############### TESTERS ####################

add_test(testEventLogHelp eventlog -h)
add_test(testAnalyzeInterceptHelp analyze_intercept.py -h)
add_test(testAnalyzeLocalizationHelp analyze_localization.py -h)
add_test(testAnalyzeMotionHelp analyze_motion.py -h)
add_test(testMonitorMotionHelp monitor_motion.py -h)
add_test(testMonitorInterceptHelp monitor_intercept.py -h)
#add_test(testKSTPlotPPHelp kstplot_pp.py -h) # pykst not supported in python3


