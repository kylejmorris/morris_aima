/**
* CLASS: TileGrid
* DATE: 05/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A Grid contains tiles, that's about it. Those tiles contain more things inside them.
* The locations of a TileGrid are cartesian, with the top left being (0,0) and bottom right being the max.
* The grid is 0-indexed, to follow standards.
*/

#ifndef MORRIS_AIMA_TILEGRID_H
#define MORRIS_AIMA_TILEGRID_H

#include <vector>
#include "Entity.h"
#include "TileLocation.h"

class Tile;
class TileGrid {
private:
    //Width and height of grid. Once set this doesn't change.
    const int WIDTH;
    const int HEIGHT;
    std::vector<std::vector<Tile> > tiles;
public:
    TileGrid(int width, int height);
    ~TileGrid();

    /**
     * Return all entities on a given tile.
     * @param location: The TileLocation to get entity at
     * @return vector<Entity *>: List of entities on this tile.
     */
    std::vector<Entity *> getEntitiesAt(TileLocation *location);

    /**
     * Add an entity to tile at position (x,y)
     * @param location: The TileLocation to place entity at
     * @param e: Entity to add
     * return bool: True if entity was added successfully, false otherwise
     */
    bool add(Entity *e, TileLocation *location);

    /**
     * Remove entity with given ID from the grid.
     * @param id: The id of entity to remove.
     * @return Entity: The entity that was removed. NULL if it doesn't exist in grid.
     */
    Entity * remove(int id);

    /**
     * Get the location of entity with given ID.
     * @param id: the id of entity to get location of
     * return TileLocation: The location of this entity.
     */
    TileLocation * getLocation(int id);
};

#endif //MORRIS_AIMA_TILEGRID_H
