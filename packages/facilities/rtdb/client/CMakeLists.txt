cmake_minimum_required(VERSION 3.5.1)

project(RtDBclient)

# this sub-library exports a single header
add_library(${PROJECT_NAME} INTERFACE)
add_library(RtDB::client ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME}
    INTERFACE
        ${PROJECT_SOURCE_DIR}/include
)

