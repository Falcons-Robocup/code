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

#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_library(yocto src/yapi/yapi.c src/yapi/yfifo.c src/yapi/yhash.c src/yapi/yjni.c src/yapi/yjson.c src/yapi/ykey.c src/yapi/ymemory.c src/yapi/ypkt_lin.c src/yapi/ypkt_osx.c src/yapi/ypkt_win.c src/yapi/yprog.c src/yapi/ystream.c src/yapi/ytcp.c src/yapi/ythread.c src/yocto_accelerometer.cpp src/yocto_files.cpp src/yocto_pwminput.cpp src/yocto_altitude.cpp src/yocto_genericsensor.cpp src/yocto_pwmoutput.cpp src/yocto_anbutton.cpp src/yocto_gps.cpp src/yocto_pwmpowersource.cpp src/yocto_api.cpp src/yocto_groundspeed.cpp src/yocto_quadraturedecoder.cpp src/yocto_audioin.cpp src/yocto_gyro.cpp src/yocto_realtimeclock.cpp src/yocto_audioout.cpp src/yocto_hubport.cpp src/yocto_refframe.cpp src/yocto_bluetoothlink.cpp src/yocto_humidity.cpp src/yocto_relay.cpp src/yocto_buzzer.cpp src/yocto_latitude.cpp src/yocto_segmenteddisplay.cpp src/yocto_carbondioxide.cpp src/yocto_led.cpp src/yocto_serialport.cpp src/yocto_cellular.cpp src/yocto_lightsensor.cpp src/yocto_servo.cpp src/yocto_colorled.cpp src/yocto_longitude.cpp src/yocto_temperature.cpp src/yocto_compass.cpp src/yocto_magnetometer.cpp src/yocto_tilt.cpp src/yocto_current.cpp src/yocto_motor.cpp src/yocto_voc.cpp src/yocto_currentloopoutput.cpp src/yocto_network.cpp src/yocto_voltage.cpp src/yocto_datalogger.cpp src/yocto_oscontrol.cpp src/yocto_wakeupmonitor.cpp src/yocto_digitalio.cpp src/yocto_power.cpp src/yocto_wakeupschedule.cpp src/yocto_display.cpp src/yocto_poweroutput.cpp src/yocto_watchdog.cpp src/yocto_dualpower.cpp src/yocto_pressure.cpp src/yocto_wireless.cpp)
target_include_directories(yocto PRIVATE ${PROJECT_SOURCE_DIR}/include/ext/ ${PROJECT_SOURCE_DIR}/include/ext/yapi/)
target_link_libraries(yocto m usb-1.0 pthread)
