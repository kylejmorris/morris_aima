#include "RandomReflexVacuumAgent.h"
#include "VacuumPercept.h"
#include "RandomVacuumAction.h"

int RandomReflexVacuumAgent::compareTo(Entity *other) {
    return Agent::compareTo(other);
}

std::string RandomReflexVacuumAgent::toString() {
    return "RandomReflexVacuumAgent";
}

std::string RandomReflexVacuumAgent::getType() {
    return "RandomReflexVacuumAgent";
}

Action *RandomReflexVacuumAgent::think(Percept *given) {
    VacuumPercept *percept = dynamic_cast<VacuumPercept *>(given);
    RandomVacuumAction *decision = NULL;

    if(percept->isDirty()) {
        decision=new RandomVacuumAction(RandomVacuumAction::SUCK);
    } else {
        //randomly pick a direction to move in
        int decisionNum = rand()%4; //random value to select which decision to make between 0 and 3
        switch(decisionNum) {
            case 0: decision = new RandomVacuumAction(RandomVacuumAction::DOWN); break;
            case 1: decision = new RandomVacuumAction(RandomVacuumAction::UP); break;
            case 2: decision = new RandomVacuumAction(RandomVacuumAction::LEFT); break;
            case 3: decision = new RandomVacuumAction(RandomVacuumAction::RIGHT); break;
        }
    }

    return decision;
}
