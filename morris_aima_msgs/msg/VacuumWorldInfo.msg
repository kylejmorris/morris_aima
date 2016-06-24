#The state info of a VacuumWorld at a given point in time.
Header header

#Whether or not the environment has been given a configuration.o
#We can't start a VacuumWorld that isn't loaded.
bool loaded

#Number of cycles the environment has gone through (active transitions)
uint32 age

#Number of columns for grid
uint16 width

#number of rows for grid
uint16 height

#performance measure
int32 performance_measure

#list of entities on the grid, and their locations.
TileEntityInfo[] entities
