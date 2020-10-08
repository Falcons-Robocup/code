cmake_minimum_required(VERSION 3.5.1)

project(sharedTypes)

find_package(Facilities)

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#include_directories(${PROJECT_SOURCE_DIR} include)

#Create library libFalconsTypes
add_library(FalconsTypes src/objectMeasurement.cpp src/uniqueObjectID.cpp)

add_custom_target(
    enum2str ALL
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/generateEnums.py -v -i ${CMAKE_CURRENT_SOURCE_DIR}/include -o ${CMAKE_CURRENT_SOURCE_DIR}/include/generated_enum2str.hpp -p ${CMAKE_CURRENT_SOURCE_DIR}/src/sharedTypes.py
    BYPRODUCTS generated_enum2str.hpp sharedTypes.py
    COMMENT "Generating code from enums"
)

add_dependencies(FalconsTypes enum2str)

