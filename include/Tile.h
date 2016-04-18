/**
* CLASS: Tile
* DATE: 07/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A tile is contained within a grid. It may contain Entities; but only 1 of each type.
*/

#ifndef MORRIS_AIMA_TILE_H
#define MORRIS_AIMA_TILE_H
#include <vector>


class Entity;

class Tile {
private:
    std::vector<Entity *>contents;
public:
    Tile();
    ~Tile();

    /**
     * Add an entity to this tile.
     * @param target: the entity to add to this tile.
     * @return bool: True if entity was added. False otherwise (ie already on the tile)
     */
    bool addEntity(Entity *target);

    /**
     * Return contents of the tile.
     */
    std::vector<Entity *>getContents();

    /**
     * Search for Entity with a given id.
     * @param id: the id of entity to search for in tile
     * @return bool: true if Entity was found, false otherwise.
     */
    bool exists(int id);

    /**
     * Remove entity with given ID from tile.
     * @param id: Id of entity to remove
     * @return: Entity that was removed. NULL if no such entity.
     */
    Entity * removeEntity(int id);
};


#endif //MORRIS_AIMA_TILE_H
