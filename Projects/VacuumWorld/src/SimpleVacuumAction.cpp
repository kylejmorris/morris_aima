#include "SimpleVacuumAction.h"

SimpleVacuumAction::SimpleVacuumAction(enum SimpleVacuumAction::ACTION choice) {
    this->choice = choice;
}

std::string SimpleVacuumAction::toString() {
    std::string result;

    if(SimpleVacuumAction::choice == SimpleVacuumAction::ACTION::LEFT) {
        result = "SimpleVacuumAction::LEFT";
    } else if(SimpleVacuumAction::choice == SimpleVacuumAction::ACTION::RIGHT) {
        result = "SimpleVacuumAction::RIGHT";
    } else {
        result = "SimpleVacuumAction::SUCK";
    }

    return result;
}
