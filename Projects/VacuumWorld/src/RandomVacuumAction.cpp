//
// Created by votick on 19/05/16.
//

#include "RandomVacuumAction.h"

std::string RandomVacuumAction::toString() {
    std::string result;

    if(RandomVacuumAction::choice==RandomVacuumAction::ACTION::UP) {
        result = "RandomVacuumAction::UP";
    } else if(RandomVacuumAction::choice==RandomVacuumAction::ACTION::DOWN) {
        result = "RandomVacuumAction::DOWN";
    } else if(RandomVacuumAction::choice==RandomVacuumAction::ACTION::LEFT) {
        result = "RandomVacuumAction::LEFT";
    } else if(RandomVacuumAction::choice==RandomVacuumAction::ACTION::RIGHT) {
        result = "RandomVacuumAction::RIGHT";
    } else {
        result = "RandomVacuumAction::SUCK";
    }

    return result;
}

RandomVacuumAction::RandomVacuumAction(enum RandomVacuumAction::ACTION choice) {
    this->choice = choice;
}
