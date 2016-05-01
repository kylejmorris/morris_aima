#include "VacuumAction.h"

VacuumAction::VacuumAction(enum VacuumAction::ACTION choice) {
    this->choice = choice;
}

std::string VacuumAction::toString() {
    std::string result;

    if(VacuumAction::choice==VacuumAction::ACTION::LEFT) {
        result = "VacuumAction::LEFT";
    } else if(VacuumAction::choice==VacuumAction::ACTION::RIGHT) {
        result = "VacuumAction::RIGHT";
    } else {
        result = "VacuumAction::SUCK";
    }

    return result;
}
