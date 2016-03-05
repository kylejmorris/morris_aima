/**
* CLASS: TileGrid
* DATE: 05/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A Grid contains tiles, that's about it. Those tiles contain more things inside them.
*/

#ifndef MORRIS_AIMA_TILEGRID_H
#define MORRIS_AIMA_TILEGRID_H

#include <vector>
#include "Entity.h"

class TileGrid {
private:
    //Width and height of grid. Once set this doesn't change.
    const int WIDTH;
    const int HEIGHT;

public:
    TileGrid(int width, int height);
    std::vector<Entity *>

};

#endif //MORRIS_AIMA_TILEGRID_H
