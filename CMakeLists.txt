cmake_minimum_required(VERSION 3.3)
#PATHS
set(MORRIS_AIMA_PATH $ENV{HOME}/workspace/morris_aima)
set(VISUALIZER_PATH ${MORRIS_AIMA_PATH}/visualizer)
set(PROJECT_PATH ${MORRIS_AIMA_PATH}/Projects)
#INCLUDES
include_directories("core/include")
include_directories("visualizer/include")
include_directories("include")
include_directories("Projects/VacuumWorld/include")
include_directories(${VISUALIZER_PATH}/include)
include_directories("$ENV{HOME}/libs")
include_directories("${PROJECT_PATH}/MissionariesAndCannibals/include")

project(morris_aima)
set(CMAKE_AUTOMOC ON)
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
###########################
# OPENGL/GLUT PACKAGES
###########################
FIND_PACKAGE(OpenGL)
IF (OPENGL_FOUND)
	MESSAGE("OPENGL FOUND")
	MESSAGE(${OPENGL_LIBRARIES})
ELSE (OPENGL_FOUND)
	MESSAGE("OPENGL NOT FOUND")
	SET(OPENGL_gl_LIBRARY opengl32)
	SET(OPENGL_glu_LIBRARY glu32)
ENDIF (OPENGL_FOUND)

#########################################################
# FIND GLUT
#########################################################
#find_package(GLUT)
#include_directories(${GLUT_INCLUDE_DIRS})
#link_directories(${GLUT_LIBRARY_DIRS})
#add_definitions(${GLUT_DEFINITIONS})
#if(NOT GLUT_FOUND)
#	message(ERROR " GLUT not found!")
#endif(NOT GLUT_FOUND)
#########################################################
# FIND OPENGL
#########################################################
#find_package(OpenGL REQUIRED)
#include_directories(${OpenGL_INCLUDE_DIRS})
#link_directories(${OpenGL_LIBRARY_DIRS})
#add_definitions(${OpenGL_DEFINITIONS})
#if(NOT OPENGL_FOUND)
#	message(ERROR " OPENGL not found!")
#endif(NOT OPENGL_FOUND)

#JSONCPP support
set(jsoncpp_ROOT "$ENV{HOME}/libs/jsoncpp")
message(JSON dir: ${jsoncpp_ROOT})
find_library(jsoncpp NAMES jsoncpp)
include_directories(${jsoncpp_ROOT}/include)
link_directories(${jsoncpp_ROOT}/lib)
set(JSON_CPP $ENV{HOME}/libs/json/jsoncpp.cpp
             $ENV{HOME}/libs/json/json.h
             $ENV{HOME}/libs/json/json-forwards.h)

#########################
#QT MODULES
##########################
find_package( Qt5Core REQUIRED )
find_package( Qt5Widgets REQUIRED )
find_package( Qt5Gui REQUIRED )
#---------------------------

#PATHS TO INCLUDES/SRC
set(CORE src/Agent.cpp core/include/Agent.h
		src/Simulator.cpp include/Simulator.h
        src/Entity.cpp core/include/Entity.h
        src/Environment.cpp core/include/Environment.h
        src/EnvironmentState.cpp core/include/EnvironmentState.h
        src/Location.cpp core/include/Location.h
        src/TileEnvironment.cpp include/TileEnvironment.h
        src/TileEnvironmentState.cpp include/TileEnvironmentState.h
        src/TileGrid.cpp include/TileGrid.h
        src/TileLocation.cpp include/TileLocation.h
        src/EnvironmentStateFactory.cpp include/EnvironmentStateFactory.h
        src/Tile.cpp include/Tile.h core/include/Action.h core/include/Percept.h
        core/include/PerformanceMeasure.h
        core/include/SimulatorResult.h src/SimulatorResult.cpp
        src/EntityFactoryFactory.cpp include/EntityFactoryFactory.h src/SimulatorFactory.cpp include/SimulatorFactory.h include/SimulatorResultFactory.h src/SimulatorResultFactory.cpp)

#VISUALIZER SUPPORT
set(VISUALIZER_SRC ${VISUALIZER_PATH}/src/FrameVisualizer.cpp
		${VISUALIZER_PATH}/src/TileFrameVisualizer.cpp
		${VISUALIZER_PATH}/src/VisualEntity.cpp
		${VISUALIZER_PATH}/src/VisualEntityFactory.cpp
		${VISUALIZER_PATH}/src/VisualImageEntity.cpp
		${VISUALIZER_PATH}/src/VisualShapeEntity.cpp
		${VISUALIZER_PATH}/src/VisualTile.cpp
		${VISUALIZER_PATH}/src/VisualTileGrid.cpp
		${VISUALIZER_PATH}/src/TerminalTextVisualizer.cpp ${VISUALIZER_PATH}/include/TerminalTextVisualizer.h)

add_library(visualizer STATIC ${VISUALIZER_SRC} )
qt5_use_modules(visualizer Core Widgets Gui )

###########################
#VACUUM WORLD PROJECT
###########################
set(VACUUM_WORLD_CORE
		Projects/VacuumWorld/src/VacuumEnvironment.cpp Projects/VacuumWorld/include/VacuumEnvironment.h
		Projects/VacuumWorld/src/SimpleReflexVacuumAgent.cpp Projects/VacuumWorld/include/SimpleReflexVacuumAgent.h
		Projects/VacuumWorld/src/DirtEntity.cpp Projects/VacuumWorld/include/DirtEntity.h
		Projects/VacuumWorld/src/VacuumPercept.cpp Projects/VacuumWorld/include/VacuumPercept.h
		Projects/VacuumWorld/src/SimpleVacuumAction.cpp Projects/VacuumWorld/include/SimpleVacuumAction.h
		Projects/VacuumWorld/src/VacuumWorldPerformanceMeasure.cpp Projects/VacuumWorld/include/VacuumWorldPerformanceMeasure.h
		Projects/VacuumWorld/src/VacuumFactory.cpp Projects/VacuumWorld/include/VacuumFactory.h
		Projects/VacuumWorld/src/VacuumEnvironmentState.cpp Projects/VacuumWorld/include/VacuumEnvironmentState.h
		Projects/VacuumWorld/src/VacuumWorldSimulationResult.cpp Projects/VacuumWorld/src/VacuumFactory.cpp
		Projects/VacuumWorld/include/VacuumFactory.h Projects/VacuumWorld/src/VacuumEnvironmentState.cpp
		Projects/VacuumWorld/include/VacuumEnvironmentState.h Projects/VacuumWorld/include/VacuumWorldSimulationResult.h
		Projects/VacuumWorld/src/VacuumWorldSimulationResult.cpp
		Projects/VacuumWorld/src/WallEntity.cpp Projects/VacuumWorld/include/WallEntity.h Projects/VacuumWorld/src/VacuumAgent.cpp Projects/VacuumWorld/include/VacuumAgent.h Projects/VacuumWorld/src/RandomReflexVacuumAgent.cpp Projects/VacuumWorld/include/RandomReflexVacuumAgent.h Projects/VacuumWorld/src/RandomVacuumAction.cpp Projects/VacuumWorld/include/RandomVacuumAction.h)

add_executable(VacuumWorld Projects/VacuumWorld/src/VacuumWorld_simple_DEMO.cpp ${CORE} ${VACUUM_WORLD_CORE} ${JSON_CPP})
target_link_libraries(VacuumWorld visualizer "jsoncpp")
qt5_use_modules( VacuumWorld Core Widgets Gui )

#randomized reflex vacuum world
add_executable(Randomized_VacuumWorld Projects/VacuumWorld/src/Randomized_VacuumWorld_DEMO.cpp ${CORE} ${VACUUM_WORLD_CORE} ${JSON_CPP})
target_link_libraries(Randomized_VacuumWorld visualizer "jsoncpp")
qt5_use_modules(Randomized_VacuumWorld Core Widgets Gui )

################################
# MISSIONARIES AND CANNIBALS PROJECT
################################
set(MISSIONARIES_AND_CANNIBALS_CORE
		Projects/MissionariesAndCannibals/src/MandCEnvironmentState.cpp Projects/MissionariesAndCannibals/include/MandCEnvironmentState.h
		Projects/MissionariesAndCannibals/src/MandCEnvironment.cpp Projects/MissionariesAndCannibals/include/MandCEnvironment.h Projects/MissionariesAndCannibals/src/StateNode.cpp Projects/MissionariesAndCannibals/include/StateNode.h Projects/MissionariesAndCannibals/src/MandCAction.cpp Projects/MissionariesAndCannibals/include/MandCAction.h
		Projects/MissionariesAndCannibals/src/MissionariesAndCannibals_DEMO.cpp)
add_executable(missionaries_and_cannibals ${MISSIONARIES_AND_CANNIBALS_CORE} ${VACUUM_WORLD_CORE} ${CORE} ${JSON_CPP}) #the dependences are so dumb here. Since factories require all the vacuum stuff, I have to include everything here. I'll fix this if/when I migrate to ros.
target_link_libraries(missionaries_and_cannibals visualizer "jsoncpp")
qt5_use_modules(missionaries_and_cannibals Core Widgets Gui )
#TODO organize cmake files into more specific looks
#---------------------------
add_subdirectory(testing)
