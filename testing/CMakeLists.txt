cmake_minimum_required(VERSION 3.3)
project(testing)

#all the test files

set(TEST_FILES Entity_Tests.cpp Tile_Tests.cpp TileEnvironment_Tests.cpp ../core/Visualizer.cpp ../core/Visualizer.h)

set(JSON_CPP $ENV{HOME}/libs/jsoncpp.cpp
             $ENV{HOME}/libs/json/json.h
             $ENV{HOME}/libs/json/json-forwards.h)
add_executable(unit_tests main.cpp ${TEST_FILES} ${JSON_CPP}
        ../core/Agent.h ../core/Agent.cpp
        ../core/Entity.h ../core/Entity.cpp
        ../core/Tile.h ../core/Tile.cpp
        ../core/TileEnvironmentState.h ../core/TileEnvironmentState.cpp
        ../core/TileEnvironment.h ../core/TileEnvironment.cpp
        ../core/TileLocation.h ../core/TileLocation.cpp
        ../core/Environment.cpp ../core/Environment.h
        ../core/EnvironmentState.cpp ../core/EnvironmentState.h
        ../core/Location.cpp ../core/Location.h
        ../core/TileEnvironment.cpp ../core/TileEnvironment.h
        ../core/TileEnvironmentState.cpp ../core/TileEnvironmentState.h
        ../core/TileGrid.cpp ../core/TileGrid.h
        ../core/TileLocation.cpp ../core/TileLocation.h
        ../core/Tile.cpp ../core/Tile.h)

target_link_libraries(unit_tests gtest_main gtest)

