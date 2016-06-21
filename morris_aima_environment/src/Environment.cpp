#include "Environment.h"

long Environment::getAge() {
    return this->age;
}

void Environment::cycle() {
    if(isActive()) {
        act();
        generate();
        updateResults();

        //update environments age
        this->age++;
    }
}

bool Environment::isActive() {
    return this->active;
}

void Environment::activate() {
    this->active = true;
}

void Environment::deactivate() {
    this->active = false;
}
