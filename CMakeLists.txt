cmake_minimum_required(VERSION 3.3)

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
include_directories("$ENV{HOME}/libs")
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

#MAIN FILES/INCLUDES TO USE#
include_directories("core/include")
include_directories("visualizer/include")
include_directories("include")
include_directories("Projects/VacuumWorld/include")

#PATHS TO INCLUDES/SRC
set(CORE src/Agent.cpp core/include/Agent.h
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
        src/EntityFactoryFactory.cpp include/EntityFactoryFactory.h src/SimulatorFactory.cpp include/SimulatorFactory.h src/SimulatorResultFactory.cpp include/SimulatorResultFactory.h)

set(VISUALIZER
		src/Visualizer.cpp
		visualizer/src/FrameVisualizer.cpp visualizer/include/FrameVisualizer.h
		src/Simulator.cpp include/Simulator.h
		visualizer/src/VisualTileGrid.cpp visualizer/include/VisualTileGrid.h
		visualizer/src/VisualEntity.cpp visualizer/include/VisualEntity.h
		core/include/Visualizer.h visualizer/src/TileFrameVisualizer.cpp
		visualizer/include/TileFrameVisualizer.h visualizer/src/VisualTile.cpp
		visualizer/include/VisualTile.h visualizer/src/VisualEntityFactory.cpp
		visualizer/include/VisualEntityFactory.h visualizer/src/VisualShapeEntity.cpp
		visualizer/include/VisualImageEntity.h visualizer/src/VisualImageEntity.cpp
		visualizer/include/VisualShapeEntity.h)

set(VACUUM_WORLD_PROJECT
		Projects/VacuumWorld/src/VacuumEnvironment.cpp Projects/VacuumWorld/include/VacuumEnvironment.h
		Projects/VacuumWorld/src/VacuumAgent.cpp Projects/VacuumWorld/include/VacuumAgent.h
		Projects/VacuumWorld/src/DirtEntity.cpp Projects/VacuumWorld/include/DirtEntity.h
		Projects/VacuumWorld/src/VacuumPercept.cpp Projects/VacuumWorld/include/VacuumPercept.h
		Projects/VacuumWorld/src/VacuumAction.cpp Projects/VacuumWorld/include/VacuumAction.h
		Projects/VacuumWorld/src/VacuumWorldPerformanceMeasure.cpp Projects/VacuumWorld/include/VacuumWorldPerformanceMeasure.h
		Projects/VacuumWorld/src/VacuumFactory.cpp Projects/VacuumWorld/include/VacuumFactory.h
		Projects/VacuumWorld/src/VacuumEnvironmentState.cpp Projects/VacuumWorld/include/VacuumEnvironmentState.h
		Projects/VacuumWorld/src/VacuumWorldSimulationResult.cpp Projects/VacuumWorld/src/VacuumFactory.cpp
		Projects/VacuumWorld/include/VacuumFactory.h Projects/VacuumWorld/src/VacuumEnvironmentState.cpp
		Projects/VacuumWorld/include/VacuumEnvironmentState.h Projects/VacuumWorld/include/VacuumWorldSimulationResult.h
		Projects/VacuumWorld/src/VacuumWorldSimulationResult.cpp)

set(SOURCE_FILES main.cpp ${CORE} ${JSON_CPP})
#TODO organize cmake files into more specific looks
#---------------------------
add_subdirectory(testing)

add_executable(morris_aima ${SOURCE_FILES} ${VISUALIZER} ${VACUUM_WORLD_PROJECT})
qt5_use_modules( morris_aima Core Widgets Gui )
