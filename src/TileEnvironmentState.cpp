#include "TileEnvironmentState.h"

bool TileEnvironmentState::isValid() {
    return true;
}

int TileEnvironmentState::compareTo(EnvironmentState *other) {
    return 0;
}

//TODO add bounds checking/testing
bool TileEnvironmentState::add(Entity *e, Location *location) {
    bool result = false;
    result = tiles->add(e, static_cast<TileLocation *>(location));
    return result;
}

TileLocation *TileEnvironmentState::getLocationOf(int id) {
    return this->tiles->getLocation(id);
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

int TileEnvironmentState::getWidth() {
    return this->WIDTH;
}

int TileEnvironmentState::getHeight() {
    return this->HEIGHT;
}

TileEnvironmentState::TileEnvironmentState(int width, int height) : WIDTH(width), HEIGHT(height) {
    this->tiles = new TileGrid(width, height);
}

std::vector<Entity *> TileEnvironmentState::getEntitiesAt(TileLocation *place) {
    return this->tiles->getEntitiesAt(place);
}

std::vector<Entity *> TileEnvironmentState::getEntities() {
    std::vector<Entity *> result;
    std::vector<Entity *> onTile; //entities on a specific tile.
    TileLocation loc(0, 0);

    for (int y = 0; y < this->getHeight(); y++) {
        for (int x = 0; x < this->getWidth(); x++) {
            loc = TileLocation(x, y); //current tile we're checking
            onTile = getEntitiesAt(&loc);
            result.insert(result.end(), onTile.begin(), onTile.end());
        }
    }

    return result;
}

bool TileEnvironmentState::moveEntity(int id, TileLocation *newLocation) {
    Entity *target;
    bool result = false;

    if(exists(id) && newLocation->getX()<=getWidth() && newLocation->getY()<=getHeight()) {
        target = remove(id);
        add(target, newLocation);
        result = true;
    }

    return result;
}
