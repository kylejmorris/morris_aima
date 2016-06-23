cmake_minimum_required(VERSION 3.0)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
project(morris_aima_environment)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


## Uncomment this if the package has a setup.py. This macro ensures
## modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
# catkin_python_setup()

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
# add_message_files(
#   FILES
#   Message1.msg
#   Message2.msg
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

## To declare and build dynamic reconfigure parameters within this
## package, follow these steps:
## * In the file package.xml:
##   * add a build_depend and a run_depend tag for "dynamic_reconfigure"
## * In this file (CMakeLists.txt):
##   * add "dynamic_reconfigure" to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * uncomment the "generate_dynamic_reconfigure_options" section below
##     and list every .cfg file to be processed

## Generate dynamic reconfigure parameters in the 'cfg' folder
# generate_dynamic_reconfigure_options(
#   cfg/DynReconf1.cfg
#   cfg/DynReconf2.cfg
# )

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES morris_aima_environment
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
# add_library(morris_aima_environment
#   src/${PROJECT_NAME}/morris_aima_environment.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(morris_aima_environment ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(morris_aima_environment_node src/morris_aima_environment_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(morris_aima_environment_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(morris_aima_environment_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS morris_aima_environment morris_aima_environment_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_morris_aima_environment.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

#JSONCPP support
set(jsoncpp_ROOT "$ENV{HOME}/libs/jsoncpp")
message(JSON dir: ${jsoncpp_ROOT})
find_library(jsoncpp NAMES jsoncpp)
include_directories(${jsoncpp_ROOT}/include)
link_directories(${jsoncpp_ROOT}/lib)
set(JSON_CPP $ENV{HOME}/libs/json/jsoncpp.cpp
        $ENV{HOME}/libs/json/json.h
        $ENV{HOME}/libs/json/json-forwards.h)

include_directories(include include/TileEnvironment include/TileEnvironment/VacuumWorld ${catkin_INCLUDE_DIRS})

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
#VARIABLES FOR ENVIRONMENT FILES
#would make sense to just make this a library
set(ENVIRONMENT_CORE
        src/Agent.cpp
        src/Entity.cpp include/Entity.h
        src/Location.cpp
        src/EnvironmentState.cpp
        src/EntityFactory.cpp include/EntityFactory.h
        src/EntityFactoryFactory.cpp include/EntityFactoryFactory.h
        src/EnvironmentStateFactory.cpp include/EnvironmentStateFactory.h
        src/Environment.cpp include/Environment.h
        src/EnvironmentFactory.cpp
        )
#really this could be a library
set(TILE_ENVIRONMENT_SRC
        src/TileEnvironment/TileEnvironment.cpp include/TileEnvironment/TileEnvironment.h
        src/TileEnvironment/Tile.cpp include/TileEnvironment/Tile.h
        src/TileEnvironment/TileEnvironmentState.cpp include/TileEnvironment/TileEnvironmentState.h
        src/TileEnvironment/TileGrid.cpp include/TileEnvironment/TileGrid.h
        src/TileEnvironment/TileLocation.cpp include/TileEnvironment/TileLocation.h
        )

set(VACUUM_WORLD_SRC
        src/TileEnvironment/VacuumWorld/DirtEntity.cpp
        src/TileEnvironment/VacuumWorld/RandomReflexVacuumAgent.cpp
        src/TileEnvironment/VacuumWorld/RandomVacuumAction.cpp
        src/TileEnvironment/VacuumWorld/SimpleReflexVacuumAgent.cpp
        src/TileEnvironment/VacuumWorld/SimpleVacuumAction.cpp
        src/TileEnvironment/VacuumWorld/VacuumAgent.cpp
        src/TileEnvironment/VacuumWorld/VacuumEnvironment.cpp
        src/TileEnvironment/VacuumWorld/VacuumEnvironmentState.cpp
        src/TileEnvironment/VacuumWorld/VacuumFactory.cpp
        src/TileEnvironment/VacuumWorld/VacuumPercept.cpp
        src/TileEnvironment/VacuumWorld/VacuumWorldPerformanceMeasure.cpp
        src/TileEnvironment/VacuumWorld/WallEntity.cpp
        include/TileEnvironment/VacuumWorld/DirtEntity.h
        include/TileEnvironment/VacuumWorld/RandomReflexVacuumAgent.h
        include/TileEnvironment/VacuumWorld/RandomVacuumAction.h
        include/TileEnvironment/VacuumWorld/SimpleReflexVacuumAgent.h
        include/TileEnvironment/VacuumWorld/SimpleVacuumAction.h
        include/TileEnvironment/VacuumWorld/VacuumAgent.h
        include/TileEnvironment/VacuumWorld/VacuumEnvironment.h
        include/TileEnvironment/VacuumWorld/VacuumEnvironmentState.h
        include/TileEnvironment/VacuumWorld/VacuumFactory.h
        include/TileEnvironment/VacuumWorld/VacuumPercept.h
        include/TileEnvironment/VacuumWorld/VacuumWorldPerformanceMeasure.h
        include/TileEnvironment/VacuumWorld/WallEntity.h
)

add_executable(morris_aima_environment_node src/morris_aima_environment_node.cpp ${ENVIRONMENT_CORE} ${TILE_ENVIRONMENT_SRC} ${VACUUM_WORLD_SRC})
target_link_libraries(morris_aima_environment_node ${catkin_LIBRARIES} "jsoncpp")
