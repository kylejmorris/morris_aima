A Tile Frame Visualizer displays a grid of tiles in 2 dimensions and displays the results in a QT frame.

PARAMETERS:
    Configuration is acquired from /morris_aima_environment/config, the visualizer will access parameters (outlined here)[LINK TO PARAMETERS]. Currently, there is not support for increasing the grid size once the visualizer is running.

SUBSCRIPTIONS
/morris_aima_environment/tile_environment_info (morris_aima_msgs/TileEnvironmentInfo)

SERVICES
/morris_aima_visualizer/set_parameters
    Set up a grid with a specific width/height. This is required before the visualizer can receive environment info and display it.

/morris_aima_visualizer/start
    Allows display to update as visualizer listens to the environment node. 
    PRECONDITION:
    set_parameters service must first be called to setup environemnt width/height, or else segfaults will occur if you try to start it prematurely.
