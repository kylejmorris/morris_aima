cmake_minimum_required(VERSION 3.3)
project(testing)

#all the test files

set(TEST_FILES Entity_Tests.cpp Tile_Tests.cpp)

add_executable(unit_tests main.cpp ${TEST_FILES} ../core/Agent.h ../core/Agent.cpp ../core/Entity.h ../core/Entity.cpp ../core/Tile.h ../core/Tile.cpp)
target_link_libraries(unit_tests gtest_main gtest)

