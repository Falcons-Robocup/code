cmake_minimum_required(VERSION 3.5.1)

project(FalconsCode)

enable_testing()

# make generated headers and other special targets available in one central location
# (like sharedTypes enum2str)
set(FALCONS_CODEGEN_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/build/codegen)
file(MAKE_DIRECTORY ${FALCONS_CODEGEN_FOLDER})
include_directories(${FALCONS_CODEGEN_FOLDER})

add_subdirectory(packages)
add_subdirectory(tools)
add_subdirectory(vision)

