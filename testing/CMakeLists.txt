cmake_minimum_required(VERSION 3.3)
project(testing)

#all the test files

set(TEST_FILES Entity_Tests.cpp Tile_Tests.cpp TileEnvironment_Tests.cpp ../src/Visualizer.cpp ../core/include/Visualizer.h)

set(JSON_CPP $ENV{HOME}/libs/json/jsoncpp.cpp
             $ENV{HOME}/libs/json/json.h
             $ENV{HOME}/libs/json/json-forwards.h)
add_executable(unit_tests main.cpp ${TEST_FILES} ${JSON_CPP}
        ../core/include/Agent.h ../src/Agent.cpp
        ../core/include/Entity.h ../src/Entity.cpp
        ../include/Tile.h ../src/Tile.cpp
        ../include/TileEnvironmentState.h ../src/TileEnvironmentState.cpp
        ../include/TileEnvironment.h ../src/TileEnvironment.cpp
        ../include/TileLocation.h ../src/TileLocation.cpp
        ../src/Environment.cpp ../core/include/Environment.h
        ../src/EnvironmentState.cpp ../core/include/EnvironmentState.h
        ../src/Location.cpp ../core/include/Location.h
        ../src/TileEnvironment.cpp ../include/TileEnvironment.h
        ../src/TileEnvironmentState.cpp ../include/TileEnvironmentState.h
        ../src/TileGrid.cpp ../include/TileGrid.h
        ../src/TileLocation.cpp ../include/TileLocation.h
        ../src/Tile.cpp ../include/Tile.h)

target_link_libraries(unit_tests gtest_main gtest)

