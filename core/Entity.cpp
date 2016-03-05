//
// Created by votick on 04/03/16.
//

#include "Entity.h"

int Entity::UNIQUE_ID = 0;

Entity::Entity() {
    Entity::UNIQUE_ID++;
    this->id = UNIQUE_ID;
}

int Entity::getId() {
    return id;
}