#include "WallEntity.h"

std::string WallEntity::toString() {
    return "Wall Entity";
}

std::string WallEntity::getType() {
    return "Wall";
}

int WallEntity::compareTo(Entity *other) {
    return 0;
}
