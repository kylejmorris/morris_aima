#include "Tile.h"
#include "Entity.h"
#include "Agent.h"

Tile::~Tile() {
}

bool Tile::addEntity(Entity *target) {
    contents.push_back(target);
}

std::vector<Entity *> Tile::getContents() {
    return contents;
}

bool Tile::exists(int id) {
    bool found = false;
    for(auto current : contents) {
        if(current->getId()==id) {
            found = true;
        }
    }
    return found;
}

Entity *Tile::removeEntity(int id) {
    int position = -1; //position of Entity to remove
    int index = 0; //index in entity as we search it
    Entity *returnEntity = NULL; //the entity we deleted

    if(exists(id)) {
        for(auto current : contents) {
            if(current!=NULL && current->getId()==id) {
                position = index;
                returnEntity = current;
            }
            index++;
        }
    }

    if(position!=-1) {
        contents.erase(contents.begin() + position);
    }

    return returnEntity;
}
