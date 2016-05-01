#include <VacuumPercept.h>
#include <iostream>
#include <VacuumAction.h>
#include "VacuumAgent.h"

std::string VacuumAgent::toString() {
 return "VacuumAgent";
}

Action *VacuumAgent::think(Percept *given) {
    VacuumPercept *percept = dynamic_cast<VacuumPercept *>(given);
    VacuumAction *decision = NULL;

    if(percept!=NULL) {
        if(percept->isDirty()) {
            decision = new VacuumAction(VacuumAction::SUCK);
        } else {
            if(percept->getTileName()=='A') {
                decision = new VacuumAction(VacuumAction::RIGHT);
            } else {
                decision = new VacuumAction(VacuumAction::LEFT);
            }
        }
    }

    return decision;
}

std::string VacuumAgent::getType() {
 return "VacuumAgent";
}
