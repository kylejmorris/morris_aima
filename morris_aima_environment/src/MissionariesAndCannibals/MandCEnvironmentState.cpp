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
    if (this->cannibalsLeft < 0 || this->cannibalsRight < 0 || this->missionariesLeft < 0 ||
        this->missionariesRight < 0) {
        result = false;
    }
    //make sure no one in the environment has just vanished, or extras appear.
    if (this->cannibalsLeft + this->cannibalsRight != 3) {
        result = false;
    }
    if (this->missionariesLeft + this->missionariesRight != 3) {
        result = false;
    }
    //are missionaries going to be killed? Can't have that now can we.
    if ((this->missionariesLeft < this->cannibalsLeft && this->missionariesLeft > 0) ||
        ((this->missionariesRight < this->cannibalsRight) && this->missionariesRight > 0)) {
        result = false;
    }

    return result;
}

MandCEnvironmentState::MandCEnvironmentState(MandCEnvironmentState &copy) {
    this->setCannibalsLeft(copy.getCannibalsLeft());
    this->setCannibalsRight(copy.getCannibalsRight());
    this->setMissionariesLeft(copy.getMissionariesLeft());
    this->setMissionariesRight(copy.getMissionariesRight());
    this->setRiverCrossed(copy.isRiverCrossed());
}

MandCEnvironmentState *MandCEnvironmentState::ship(int numCannibals, int numMissionaries) {
    MandCEnvironmentState *newState = new MandCEnvironmentState;
    MandCEnvironmentState *result = nullptr;
    newState->setRiverCrossed(this->isRiverCrossed());
    newState->setCannibalsRight(getCannibalsRight());
    newState->setCannibalsLeft(getCannibalsLeft());
    newState->setMissionariesLeft(getMissionariesLeft());
    newState->setMissionariesRight(getMissionariesRight());

    //transfer from right to left
    if(isRiverCrossed()) {
        newState->setMissionariesRight(newState->getMissionariesRight()-numMissionaries);
        newState->setMissionariesLeft(newState->getMissionariesLeft() + numMissionaries);

        newState->setCannibalsRight(newState->getCannibalsRight()-numCannibals);
        newState->setCannibalsLeft(newState->getCannibalsLeft() + numCannibals);
        newState->setRiverCrossed(!newState->isRiverCrossed());
    } else { //transfer from left to right
        newState->setMissionariesLeft(newState->getMissionariesLeft()-numMissionaries);
        newState->setMissionariesRight(newState->getMissionariesRight() + numMissionaries);

        newState->setCannibalsLeft(newState->getCannibalsLeft()-numCannibals);
        newState->setCannibalsRight(newState->getCannibalsRight() + numCannibals);

        newState->setRiverCrossed(!newState->isRiverCrossed());
    }

    //incase of state being invalid, just delete it
    if(!newState->isValid()) {
        delete newState;
        result = nullptr;
    } else {
        result = newState;
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
