cmake_minimum_required(VERSION 3.5.1)

project(mtrbc LANGUAGES CXX)

enable_testing()

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")
add_compile_options(-O3)

# system dependencies
list(INSERT CMAKE_MODULE_PATH 0 ${CMAKE_CURRENT_LIST_DIR}/cmake)

# components
add_subdirectory(src)

