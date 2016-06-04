#include "MandCEnvironmentState.h"

MandCEnvironmentState::MandCEnvironmentState() {
    this->cannibalsLeft = 3;
    this->cannibalsRight = 0;
    this->missionariesLeft = 3;
    this->missionariesRight = 0;
    this->riverCrossed = false;
}

bool MandCEnvironmentState::isValid() {
    bool result = true;
    //make sure no one in the environment has just vanished, or extras appear.
    if(this->cannibalsLeft+this->cannibalsRight!=6) {
        result = false;
    }
    if(this->missionariesLeft+this->missionariesRight!=6) {
        result = false;
    }
    //are missionaries going to be killed? Can't have that now can we.
    if(this->missionariesLeft<this->cannibalsLeft || this->missionariesRight<this->cannibalsRight) {
        result = false;
    }

    return result;
}

int MandCEnvironmentState::compareTo(EnvironmentState *other) {
    MandCEnvironmentState *otherState = dynamic_cast<MandCEnvironmentState *>(other);
    int result = 0; //equal until proven otherwise

    if(getMissionariesLeft()!=otherState->getMissionariesLeft()) {
        result++;
    }
    if(getMissionariesRight()!=otherState->getMissionariesRight()) {
        result++;
    }
    if(getCannibalsLeft()!=otherState->getCannibalsLeft()) {
        result++;
    }
    if(getCannibalsRight()!=otherState->getCannibalsRight()) {
        result++;
    }
    if(isRiverCrossed()!=otherState->isRiverCrossed()) {
        result++;
    }

    return result;
}
