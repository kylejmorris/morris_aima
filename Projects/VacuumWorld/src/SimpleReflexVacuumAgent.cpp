#include <VacuumPercept.h>
#include <iostream>
#include "SimpleVacuumAction.h"
#include "SimpleReflexVacuumAgent.h"

std::string SimpleReflexVacuumAgent::toString() {
 return "SimpleReflexVacuumAgent";
}

Action *SimpleReflexVacuumAgent::think(Percept *given) {
    VacuumPercept *percept = dynamic_cast<VacuumPercept *>(given);
    SimpleVacuumAction *decision = NULL;

    if(percept!=NULL) {
        if(percept->isDirty()) {
            decision = new SimpleVacuumAction(SimpleVacuumAction::SUCK);
        } else {
            if(percept->getTileName()=='A') {
                decision = new SimpleVacuumAction(SimpleVacuumAction::RIGHT);
            } else {
                decision = new SimpleVacuumAction(SimpleVacuumAction::LEFT);
            }
        }
    }

    return decision;
}

std::string SimpleReflexVacuumAgent::getType() {
 return "SimpleReflexVacuumAgent";
}
