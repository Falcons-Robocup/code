cmake_minimum_required(VERSION 3.5.1)

project(sharedTypes)

find_package(Facilities)

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")


# Create library libFalconsTypes
set(LIBRARY_TARGET_NAME FalconsTypes)
add_library(${LIBRARY_TARGET_NAME} src/objectMeasurement.cpp src/uniqueObjectID.cpp)


# setup the code generation build hook command
# reading material: https://crascit.com/2017/04/18/generated-sources-in-cmake-builds/
# the generated artefacts are now also stored in the build/ tree (FALCONS_CODEGEN_FOLDER)
add_custom_command(
    OUTPUT  ${FALCONS_CODEGEN_FOLDER}/generated_enum2str.hpp ${FALCONS_CODEGEN_FOLDER}/sharedTypes.py
    COMMAND ${PYTHON_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/src/generateEnums.py -i ${CMAKE_CURRENT_SOURCE_DIR}/include -o ${FALCONS_CODEGEN_FOLDER}/generated_enum2str.hpp -p ${FALCONS_CODEGEN_FOLDER}/sharedTypes.py
    DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/include/*.hpp
)

# define a custom dummy target to let everything else depend on (via FalconsTypes)
add_custom_target(enum2str
    ALL
    DEPENDS ${FALCONS_CODEGEN_FOLDER}/generated_enum2str.hpp
    DEPENDS ${FALCONS_CODEGEN_FOLDER}/sharedTypes.py
    COMMENT "Generating code from enums"
)
add_dependencies(${LIBRARY_TARGET_NAME} enum2str)

