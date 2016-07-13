#A node containing info at a relevant point in a search.

#how much it cost to get to this node
int32 path_cost

#the state contained at this node
MissionariesAndCannibalsState state

#the node we got to this node from.
MissionariesAndCannibalsNode parent
