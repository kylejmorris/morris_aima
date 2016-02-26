cmake_minimum_required(VERSION 3.3)
project(morris_aima)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(morris_aima ${SOURCE_FILES})