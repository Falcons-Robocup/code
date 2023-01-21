cmake_minimum_required(VERSION 3.5.1)

project(FalconsCode)

### Default CXX flags for all projects
# ADDITIONAL COMPILER FLAGS can be added via add_compiler_options() or target_compiler_options()
set(CMAKE_CXX_FLAGS "-std=c++17 -Wall -Werror")
set(CMAKE_C_FLAGS "-Wall -Werror")

enable_testing()

# define colors for colored text messages
string(ASCII 27 Esc)
set(ColourReset "${Esc}[m")
set(ColourBold  "${Esc}[1m")
set(Red         "${Esc}[31m")
set(Green       "${Esc}[32m")
set(Yellow      "${Esc}[33m")
set(Blue        "${Esc}[34m")
set(Magenta     "${Esc}[35m")
set(Cyan        "${Esc}[36m")
set(White       "${Esc}[37m")
set(BoldRed     "${Esc}[1;31m")
set(BoldGreen   "${Esc}[1;32m")
set(BoldYellow  "${Esc}[1;33m")
set(BoldBlue    "${Esc}[1;34m")
set(BoldMagenta "${Esc}[1;35m")
set(BoldCyan    "${Esc}[1;36m")
set(BoldWhite   "${Esc}[1;37m")

# padding to align build type message during compilation
set(PADDING_LENGTH_CMAKE_BUILD_TYPE 30)

# pad_string
# get a string of consisting str_prefix followed by str_padding_char repetitions until end location
function(pad_string output str_padding_char str_prefix end_location)
    string(LENGTH ${str_prefix} prefix_length)
    math(EXPR padding_length "${end_location} - ${prefix_length}")
    string(REPEAT ${str_padding_char} ${padding_length} str_padding)
    set(${output} "${str_prefix}${str_padding}" PARENT_SCOPE)
endfunction()

# add custom command-line -D options
#   DEBUG_PACKAGES=<list of package-names> 
set(DEBUG_PACKAGES "" CACHE STRING "Semicolon (;) separated list of packages (case-insensitive). For specified packages CMAKE_BUILD_TYPE=DEBUG is set.")
list(TRANSFORM DEBUG_PACKAGES TOLOWER) # conver package list to lower character for more convenient comparison
message(STATUS "Custom option: DEBUG_PACKAGES=${DEBUG_PACKAGES}")

# apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}") must be called in CMakeLists.txt of the package to be supported by DEBUG_PACKAGES.
# Argument CURR_BUILD_TYPE is used to display information about the used build type (overwritten in case its requested in debug)
function(apply_debug_if_in_debug_packages CURR_BUILD_TYPE)
    string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
    set(STR_BUILD_TYPE "${Cyan}${CURR_BUILD_TYPE}${ColourReset}")
    if (${PROJECT_NAME_LOWER} IN_LIST DEBUG_PACKAGES)
        set(CMAKE_BUILD_TYPE Debug PARENT_SCOPE)
        set(STR_BUILD_TYPE "${Yellow}Debug${ColourReset}")
    elseif ("${CURR_BUILD_TYPE}" STREQUAL "")
        set(STR_BUILD_TYPE "${Cyan}Default${ColourReset}")
    endif()
    pad_string(STR_PROJECT_NAME_LOWER_PADDED "." "${PROJECT_NAME_LOWER}" ${PADDING_LENGTH_CMAKE_BUILD_TYPE})
    message(STATUS "${STR_PROJECT_NAME_LOWER_PADDED}CMAKE_BUILD_TYPE = ${STR_BUILD_TYPE}")
endfunction()

# make generated headers and other special targets available in one central location
# (like sharedTypes enum2str)
set(FALCONS_CODEGEN_FOLDER ${CMAKE_CURRENT_SOURCE_DIR}/build/codegen)
file(MAKE_DIRECTORY ${FALCONS_CODEGEN_FOLDER})
include_directories(${FALCONS_CODEGEN_FOLDER})

#add_compile_options("-Wall" "-Wextra" "-Werror") # TODO: maybe better to do it via cli (fmake), but how?

add_subdirectory(packages)
# add_subdirectory(tools) # TODO: This currently depends on a proprietary package which will be added later
add_subdirectory(vision)


# Clear DEBUG_PACKAGES variable. This ensures debug packages are only used when it is specified on the command line
# NOTE: MUST be called AFTER all users of apply_debug_if_in_debug_packages are added (add_subdirectory)
unset(DEBUG_PACKAGES CACHE)
