A TileEnvironment is one type of general environment. It involves a grid of tiles, in which may contain Entities.

Since so many possible environments may be represented as a 2D grid, this abstract environment will handle some of the trivial tasks any grid would require.

Parameter structure: located at 
/morris_aima_environment/config/grid_width: integer, the width of grid.
/morris_aima_environment/config/grid_height: int, height of grid
/morris_aima_environment/config/entities: list of Entities in the environment.
An entity contains the following structure:
    {entity_type,
    location_x,
    location_y
    }

More concretely: /morris_aima_environment/config/entities = [{entity_type,location_x,location_y} {entity_type, location_x, location_y} ... ]

    
*The configuration entities will allow the environment to be reset to it's initial state,with the same map loaded. Loading a new world map will change the parameters.

PUBLICATIONS:
    publishes morris_aima_msgs/TileEnvironmentInfo messages on morris_aima_environment/tile_environment_info
   
