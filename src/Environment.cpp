#include "Environment.h"

long Environment::getAge() {
    return this->age;
}

void Environment::cycle() {
    act();
    generate();
    updateResults();

    //update environments age
    this->age++;
}
