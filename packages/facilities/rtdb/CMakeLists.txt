cmake_minimum_required(VERSION 3.5.1)

project(RtDB LANGUAGES CXX)

add_compile_options(-ftemplate-depth=1200 -Wvla)
enable_testing()


# system dependencies
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# components
add_subdirectory(comm)
add_subdirectory(config)
add_subdirectory(definitions) # TODO only for making msgpack macros available -> rename to "serialize"?
#add_subdirectory(dictionary_generator) # broken
add_subdirectory(rtdb2) # NOTE: the 2 postfix was introduced by Cambada next to their legacy rtdb API - maybe best to keep peculiar naming as is?
add_subdirectory(utils)

add_subdirectory(test)
