cmake_minimum_required(VERSION 3.5.1)

project(Facilities)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")

# components
add_subdirectory(byteArray)
add_subdirectory(common)
add_subdirectory(configuration)
add_subdirectory(diagnostics)
add_subdirectory(environment)
add_subdirectory(filters)
add_subdirectory(ftime)
add_subdirectory(geometry)
add_subdirectory(logging)
add_subdirectory(networkUDP)
add_subdirectory(rtdb)
add_subdirectory(frtdb)
add_subdirectory(sharedTypes)
add_subdirectory(tracing)


