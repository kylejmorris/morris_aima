#include <vector>
#include "TileGrid.h"
#include "TileGrid.h"
#include "Tile.h"

TileGrid::TileGrid(int width, int height) : WIDTH(width), HEIGHT(height) {
    for(int row = 0; row<height; row++) {
        std::vector<Tile> currentRow;
        for(int col=0; col<width; col++) {
            Tile curr; //current tile to push back onto row
            currentRow.push_back(curr);
        }
        tiles.push_back(currentRow);
    }
}

TileGrid::~TileGrid() {
}

std::vector<Entity *> TileGrid::getEntitiesAt(TileLocation *location) {
    int x = location->getX();
    int y = location->getY();

    return this->tiles[y][x].getContents();
}

bool TileGrid::add(Entity *e, TileLocation *location) {
    bool result = false; //false until proven otherwise
    int xPos;
    int yPos;

    if(e!=NULL && location!=NULL) {
        xPos = location->getX();
        yPos = location->getY();
        this->tiles[yPos][xPos].addEntity(e);
        result = true;
    }

    return result;
}

Entity * TileGrid::remove(int id) {
    Entity *result = NULL;
    int xPos;
    int yPos;
    TileLocation *foundSpot = getLocation(id);

    if(foundSpot!=NULL) {
        xPos = foundSpot->getX();
        yPos = foundSpot->getY();
        result = this->tiles[yPos][xPos].removeEntity(id);
    }
    return result;
}

//TODO test this
TileLocation *TileGrid::getLocation(int id) {
    int foundX = -1;
    int foundY = -1;
    TileLocation *result = NULL; //resulting location generated from routine.

    for(int currRow=0; currRow<HEIGHT; currRow++) {
        for(int currCol=0; currCol<WIDTH; currCol++) {
            if(tiles[currRow][currCol].exists(id)) {
                foundX = currCol;
                foundY = currRow;
            }
        }
    }

    if(foundX!=-1 && foundY!=-1) {
        result = new TileLocation(foundX, foundY);
    }

    return result;
}
