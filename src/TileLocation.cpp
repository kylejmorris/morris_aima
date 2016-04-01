#include "TileLocation.h"

TileLocation &TileLocation::operator=(const TileLocation &location) {
    this->xPosition = location.xPosition;
    this->yPosition = location.yPosition;
    return (*this);
}

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
