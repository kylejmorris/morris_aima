//
// Created by votick on 05/03/16.
//

#include "TileLocation.h"

TileLocation::TileLocation(int x, int y) {
    this->xPosition = x;
    this->yPosition = y;
}

int TileLocation::getX() {
    return this->xPosition;
}

int TileLocation::getY() {
    return this->yPosition;
}
