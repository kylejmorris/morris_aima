#include "TileEnvironmentState.h"

bool TileEnvironmentState::isValid() {
    return true;
}

int TileEnvironmentState::compareTo(EnvironmentState *other) {
    return 0;
}

bool TileEnvironmentState::add(Entity *e, Location *location) {
    bool result = false;
    result = tiles->add(e, static_cast<TileLocation *>(location));
    return result;
}

Entity *TileEnvironmentState::remove(int id) {
    Entity *result = NULL;
    result = tiles->remove(id);
    return result;
}

bool TileEnvironmentState::exists(int id) {
    bool result = false;
    TileLocation *location;

    location = tiles->getLocation(id);

    if(location!=NULL) {
        result = true;
    }

    return result;
}

TileEnvironmentState::~TileEnvironmentState() {
    delete tiles;
}

TileEnvironmentState::TileEnvironmentState(int width, int height) : WIDTH(width), HEIGHT(height) {
    this->tiles = new TileGrid(width, height);
}
