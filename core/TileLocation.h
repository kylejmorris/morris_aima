/**
* CLASS: TileLocation
* DATE: 05/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Specification of the location of a Tile in a TileEnvironment.
* Anything positioned within a TileEnvironment must have a location expressed in this way.
*/

#ifndef MORRIS_AIMA_TILELOCATION_H
#define MORRIS_AIMA_TILELOCATION_H

#include "Location.h"

class TileLocation : public Location {
private:
    int xPosition;
    int yPosition;
public:
    TileLocation(int x, int y);
    int getX();
    int getY();
};

#endif //MORRIS_AIMA_TILELOCATION_H
