It's easier to load an environment using a yaml file/structure rather than manually specifying the parameters.
Here is the structure you must follow to configure any TileEnvironment.

Note this is the portion required for an environment. Refer to [TODO, ADD LINK: yaml structure for morris_aima launching]

YAML STRUCTURE: will be set at  /morris_aima_environment/config
entities:
    - entity_type: SimpleReflexVacuumAgent
      location_x: INT (x coordinate)
      location_y: INT (y coordinate)
    -
    - (more entities may be added similar to as shown above.)
grid_height: INT //number of rows of tiles in grid
grid_width: INT //number of columns of tiles in grid

Example of completed config:
entities:
    - entity_type: SimpleReflexVacuumAgent
      location_x: 5
      location_y: 2
grid_height: 10
grid_width: 10

