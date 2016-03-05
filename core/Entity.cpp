//
// Created by votick on 04/03/16.
//

#include "Entity.h"

int Entity::id = 0;

Entity::Entity() {
    this->id++;
}

int Entity::getId() {
    return this->id;
}