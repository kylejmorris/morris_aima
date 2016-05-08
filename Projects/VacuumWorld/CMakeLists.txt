cmake_minimum_required(VERSION 3.3)
project(VacuumWorld)

set(CMAKE_AUTOMOC ON)
##########################
#QT MODULES
##########################
find_package( Qt5Core REQUIRED )
find_package( Qt5Widgets REQUIRED )
find_package( Qt5Gui REQUIRED )
##########################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
set(MORRIS_AIMA_PATH $ENV{HOME}/workspace/morris_aima)

# INCLUDES
include_directories(${MORRIS_AIMA_PATH}/include)
include_directories(include)
include_directories("$ENV{HOME}/libs")

#JSONCPP support
set(JSON_CPP $ENV{HOME}/libs/json/jsoncpp.cpp
             $ENV{HOME}/libs/json/json.h
             $ENV{HOME}/libs/json/json-forwards.h)

#CORE FRAMEWORK SUPPORT
include_directories(${MORRIS_AIMA_PATH}/core/include)
include_directories(${MORRIS_AIMA_PATH}/src)
set(CORE_PATH ${MORRIS_AIMA_PATH}/src)
set(CORE_SRC ${CORE_PATH}/Agent.cpp
        ${CORE_PATH}/Entity.cpp
        ${CORE_PATH}/EntityFactory.cpp
        ${CORE_PATH}/EntityFactoryFactory.cpp
        ${CORE_PATH}/Environment.cpp
        ${CORE_PATH}/EnvironmentState.cpp
        ${CORE_PATH}/EnvironmentStateFactory.cpp
        ${CORE_PATH}/Location.cpp
        ${MORRIS_AIMA_PATH}/include/Simulator.h
        ${CORE_PATH}/Simulator.cpp
        ${CORE_PATH}/SimulatorFactory.cpp
        ${CORE_PATH}/SimulatorResult.cpp
        ${CORE_PATH}/SimulatorResultFactory.cpp
        ${CORE_PATH}/Tile.cpp
        ${CORE_PATH}/TileEnvironment.cpp
        ${CORE_PATH}/TileEnvironmentState.cpp
        ${CORE_PATH}/TileGrid.cpp
        ${CORE_PATH}/TileLocation.cpp
        ${CORE_PATH}/Visualizer.cpp
        )
#add_library(core STATIC ${CORE_SRC})
#######################

#VISUALIZER SUPPORT
set(VISUALIZER_PATH ${MORRIS_AIMA_PATH}/visualizer)
include_directories(${VISUALIZER_PATH}/include)
set(VISUALIZER_SRC ${VISUALIZER_PATH}/src/FrameVisualizer.cpp
        ${VISUALIZER_PATH}/src/TileFrameVisualizer.cpp
        ${VISUALIZER_PATH}/src/VisualEntity.cpp
        ${VISUALIZER_PATH}/src/VisualEntityFactory.cpp
        ${VISUALIZER_PATH}/src/VisualImageEntity.cpp
        ${VISUALIZER_PATH}/src/VisualShapeEntity.cpp
        ${VISUALIZER_PATH}/src/VisualTile.cpp
        ${VISUALIZER_PATH}/src/VisualTileGrid.cpp
        )
add_library(visualizer STATIC ${VISUALIZER_SRC} )
qt5_use_modules(visualizer Core Widgets Gui )
###########################

set(SOURCE_FILES
        include/DirtEntity.h
        include/VacuumAction.h
        include/VacuumAgent.h
        include/VacuumEnvironment.h
        include/VacuumEnvironmentState.h
        include/VacuumFactory.h
        include/VacuumPercept.h
        include/VacuumWorldPerformanceMeasure.h
        include/VacuumWorldSimulationResult.h
        src/DirtEntity.cpp
        src/VacuumAction.cpp
        src/VacuumAgent.cpp
        src/VacuumEnvironment.cpp
        src/VacuumEnvironmentState.cpp
        src/VacuumFactory.cpp
        src/VacuumPercept.cpp
        src/VacuumWorldPerformanceMeasure.cpp src/VacuumWorldSimulationResult.cpp main.cpp)

add_executable(VacuumWorld ${SOURCE_FILES} ${JSON_CPP} ${CORE_SRC})
target_link_libraries(VacuumWorld visualizer)
Qt5_use_modules(VacuumWorld Core Widgets Gui )
