#include "DirtEntity.h"

int DirtEntity::compareTo(Entity *other) {
    int result = -1; //this entity is less than other types of entities. Dirt is the bottom of the chain, it's dirt.
    if(other->getType().compare("Dirt")) { //dirt = dirt
        result = 0;
    }
}

std::string DirtEntity::toString() {
    return "Dirt Entity";
}

std::string DirtEntity::getType() {
    return "Dirt";
}
