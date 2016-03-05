#include "TileEnvironment.h"

void TileEnvironment::loadEnvironment(string fileName) {

}

EnvironmentState *TileEnvironment::readState() {
    return nullptr;
}

bool TileEnvironment::add(Entity *e, Location *place) {
    return false;
}

Entity *TileEnvironment::remove(int id) {
    return nullptr;
}

bool TileEnvironment::exists(int id) {
    return false;
}

std::vector<Entity *> TileEnvironment::getEntities() {
    return std::vector<Entity *>();
}
