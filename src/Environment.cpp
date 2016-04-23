#include "Environment.h"

long Environment::getAge() {
    return this->age;
}

void Environment::cycle() {
    act();
}
