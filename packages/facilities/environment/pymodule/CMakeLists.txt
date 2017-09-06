cmake_minimum_required(VERSION 2.8.0)

# MOre information on Cmake support for SWIG:
# http://www.swig.org/Doc1.3/Introduction.html#Introduction_build_system
# https://cmake.org/cmake/help/v3.0/module/UseSWIG.html
# 
# MKoenen 28/12/2015
#
# set up dependencies on header files
include_directories( ${PROJECT_SOURCE_DIR}/pymodule ${PROJECT_SOURCE_DIR}/include )

#Get Python package info to build the python modules
include($ENV{ROS_ROOT}/core/rosbuild/FindPkgConfig.cmake)
pkg_check_modules( PYTHON REQUIRED python2)
include_directories( ${PYTHON_INCLUDE_DIRS} )

find_package(SWIG REQUIRED)
if (NOT  ${SWIG_FOUND} )
   MESSAGE( FATAL_ERROR "SWIG missing, please install: 'sudo apt-get install swig'" )
endif()


include(${SWIG_USE_FILE})

set(CMAKE_SWIG_FLAGS "")

#override the standard lib outfolder to the pymodule folder
#to prevent mixup of python modules with C shared libraries
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/pymodule)
#specify where to write all other swig generated module like the module .py file
set(CMAKE_SWIG_OUTDIR ${PROJECT_SOURCE_DIR}/pymodule)

# FIELD
# 20160625: enabled again by MKOE after implementing an alternative for 'list-of-maps'
set_source_files_properties(EnvironmentField.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(EnvironmentField.i PROPERTIES CMAKE_SWIG_FLAGS "-includeall")
swig_add_module( EnvironmentField python EnvironmentField.i ../src/cEnvironmentField.cpp ../src/cEnvironmentCommon.cpp )
swig_link_libraries(EnvironmentField EnvironmentField EnvironmentCommon ${PYTHON_LIBRARIES})

# BALL
set_source_files_properties(EnvironmentBall.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(EnvironmentBall.i PROPERTIES CMAKE_SWIG_FLAGS "-includeall")
swig_add_module( EnvironmentBall python EnvironmentBall.i ../src/cEnvironmentBall.cpp ../src/cEnvironmentCommon.cpp )
swig_link_libraries(EnvironmentBall EnvironmentBall EnvironmentCommon ${PYTHON_LIBRARIES})

# ROBOT
set_source_files_properties(EnvironmentRobot.i PROPERTIES CPLUSPLUS ON)
set_source_files_properties(EnvironmentRobot.i PROPERTIES CMAKE_SWIG_FLAGS "-includeall")
swig_add_module( EnvironmentRobot python EnvironmentRobot.i ../src/cEnvironmentRobot.cpp ../src/cEnvironmentCommon.cpp )
swig_link_libraries(EnvironmentRobot EnvironmentRobot EnvironmentCommon ${PYTHON_LIBRARIES})


#swig_link_libraries explanation:
# 1st EnvironmentField= name of the target python library will be prepended with underscore, so results in python module _EnvironmentField.so, used in Python with import 'EnvironmentField'
# 2nd EnvironmentField= refers to the previously build C module in lib folder: libEnvironmentField.so which is linked to the python module with same name   
# 



