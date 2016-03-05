cmake_minimum_required(VERSION 3.3)

project(morris_aima)

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in
        googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )
execute_process(COMMAND ${CMAKE_COMMAND} --build .
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}/googletest-download )

# Prevent GoogleTest from overriding our compiler/linker options
# when building with Visual Studio
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This adds
# the following targets: gtest, gtest_main, gmock
# and gmock_main
add_subdirectory(${CMAKE_BINARY_DIR}/googletest-src
        ${CMAKE_BINARY_DIR}/googletest-build)

# The gtest/gmock targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
    include_directories("${gtest_SOURCE_DIR}/include"
            "${gmock_SOURCE_DIR}/include")
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

#MAIN FILES/INCLUDES TO USE#
include_directories("core")
set(CORE core/Agent.cpp core/Agent.h
        core/Entity.cpp core/Entity.h
        core/Environment.cpp core/Environment.h
        core/EnvironmentState.cpp core/EnvironmentState.h
        core/Location.cpp core/Location.h
        core/TileEnvironment.cpp core/TileEnvironment.h
        core/TileEnvironmentState.cpp core/TileEnvironmentState.h core/TileGrid.cpp core/TileGrid.h core/TileLocation.cpp core/TileLocation.h)

#---------------------------
add_subdirectory(testing)

set(SOURCE_FILES main.cpp ${CORE})
add_executable(morris_aima ${SOURCE_FILES})
