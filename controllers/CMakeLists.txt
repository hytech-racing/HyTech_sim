cmake_minimum_required(VERSION 3.2)
project(TorqueVectoring LANGUAGES CXX)
# 

set(CMAKE_CXX_STANDARD 17)

add_library(TorqueVectoringController SHARED
    src/TorqueVectoringController.cpp
)

target_include_directories(TorqueVectoringController PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include/TorqueVectoringController>
    $<INSTALL_INTERFACE:include/TorqueVectoringController>
)