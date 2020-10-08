cmake_minimum_required(VERSION 3.5.1)

# using shared_ptr
set(CMAKE_CXX_FLAGS "-std=c++11 -Wall -Werror")

#set the default path for built executables to the "bin" directory
#set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
#set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories("inc")

# no library, all template implementations are available via header 

