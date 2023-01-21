cmake_minimum_required(VERSION 3.5.1)

project(MCBInterface)

#find_package(Boost COMPONENTS system thread REQUIRED QUIET)
find_package(Facilities)
find_package(Protobuf REQUIRED)

find_library(PROTOBUF_LIBRARY protobuf)

# support switching to debug via -DDEBUG_PACKAGES command line option
apply_debug_if_in_debug_packages("${CMAKE_BUILD_TYPE}")


#include_directories($ENV{FALCONS_DATA_PATH}/external/protobuf/include)
#link_directories($ENV{FALCONS_DATA_PATH}/external/protobuf/lib)
#include_directories($ENV{FALCONS_DATA_PATH}/external/zeromq/include)
#link_directories($ENV{FALCONS_DATA_PATH}/external/zeromq/lib)

include_directories("include")

add_library(adapters src/RTDBInputAdapter.cpp
                     src/RTDBOutputAdapter.cpp)
target_link_libraries(adapters FalconsTypes mcb)

add_library(mcb src/Rpc.cpp)
target_include_directories(mcb PUBLIC ${CMAKE_CURRENT_BINARY_DIR})
target_link_libraries(mcb zmq protobuf)

add_executable(mcbInterface src/main.cpp)
target_link_libraries(mcbInterface adapters mcb cpp_rpc pthread Tracing diagnostics)
target_link_libraries(mcbInterface FalconsTypes)


# compile the message types
set(MESSAGE_PROTOS kicker.proto
                   motion.proto
                   motors.proto
                   publish.proto)

protobuf_generate(TARGET mcb PROTOS ${MESSAGE_PROTOS} LANGUAGE cpp)


#
# TODO: Replace with the following once the PLUGIN parameter has been added to
#       the CMake function protobuf_generate (cmake v3.21.0?)
#
#
# protobuf_generate(
#     TARGET
#         cpp_rpc_gen
#     PROTOS
#         ${SERVICE_PROTOS}
#     LANGUAGE
#         cpp_rpc
#     GENERATE_EXTENSIONS
#         .rpc.pb.h
#         .rpc.pb.cc
#     PLUGIN
#         "protoc-gen-cpp_rpc=${PROJECT_SOURCE_DIR}/scripts/rpc_cpp_plugin.py"
# )

set(SERVICE_PROTOS kicker.proto
                   motion.proto
                   motors.proto)

set(SERVICE_SRCS   kicker.rpc.pb.cc
                   motion.rpc.pb.cc
                   motors.rpc.pb.cc)

add_custom_target(
    cpp_rpc_gen
    COMMAND protoc --cpp_rpc_out=${CMAKE_CURRENT_BINARY_DIR} --plugin=protoc-gen-cpp_rpc=${PROJECT_SOURCE_DIR}/scripts/rpc_cpp_plugin.py --proto_path=${PROJECT_SOURCE_DIR} ${SERVICE_PROTOS}
    BYPRODUCTS ${SERVICE_SRCS}
)

add_library(cpp_rpc ${SERVICE_SRCS})
add_dependencies(cpp_rpc cpp_rpc_gen)
target_link_libraries(cpp_rpc mcb)
