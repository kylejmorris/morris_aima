cmake_minimum_required(VERSION 3.3)
project(modern_ai)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(SOURCE_FILES main.cpp)
add_executable(modern_ai ${SOURCE_FILES})