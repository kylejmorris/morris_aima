#include "Agent.h"
#include "Entity.h"

int Agent::compareTo(Entity *other) {
    return 0;
}

std::string Agent::toString() {
    return "";
}

Action *Agent::think(Percept *given) {
    return NULL; //NOP action
}

std::string Agent::getType() {
    return "agent";
}
