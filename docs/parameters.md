When beginning a morris_aima program, the Environment, Controller, and Visualizer will share some global parameters.

morris_aima_control/world_type: This specifies the name of the project/world that is being worked on. The Environment and Visualizer will take this and use it to determine what Topic to publish/listen on.
    Supported World Types:
        vacuum_world
        missionaries_and_cannibals: The classic problem where you must get missionaries and cannibals across a river, without anyone being eaten.
