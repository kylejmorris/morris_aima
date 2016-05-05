#include <VacuumPercept.h>
#include <DirtEntity.h>
#include <VacuumAction.h>
#include "VacuumEnvironment.h"
#include "VacuumAgent.h"

VacuumEnvironment::VacuumEnvironment() : TileEnvironment() {
    this->performanceMeasure = new VacuumWorldPerformanceMeasure;
}

void VacuumEnvironment::act() {
    this->vacuum = this->state->findVacuum();
    this->vacuumLocation = this->state->getLocationOf(this->vacuum->getId());
    if(this->vacuum!=NULL) { //did we find vacuum?
        VacuumPercept *forVacuum = new VacuumPercept(vacuumLocation, this->state->isDirty(vacuumLocation->getX(), vacuumLocation->getY()));
        VacuumAction *action = (VacuumAction *)(this->vacuum->think(forVacuum));

        //respond to the action made by agent, updating environment state
        if(action->choice==VacuumAction::LEFT) {
            state->moveVacuum(this->vacuumLocation->getX()-1, this->vacuumLocation->getY());
        } else if(action->choice==VacuumAction::RIGHT) {
            state->moveVacuum(this->vacuumLocation->getX()+1, this->vacuumLocation->getY());
        } else if(action->choice==VacuumAction::SUCK) {
            state->cleanTile();
        }

        delete forVacuum;
        if(action!=NULL) {
            delete action;
        }
    }
}

void VacuumEnvironment::generate() {

}

VacuumEnvironment::~VacuumEnvironment() {
}

void VacuumEnvironment::updateResults() {
    this->performanceMeasure->update(readState());
}

EnvironmentState *VacuumEnvironment::readState() {
    return this->state;
}

void VacuumEnvironment::loadEnvironment(string fileName) {
    TileEnvironment::loadEnvironment(fileName);
    this->state = dynamic_cast<VacuumEnvironmentState *>(TileEnvironment::readState());
}

double VacuumEnvironment::getPerformanceMeasure() {
    return this->performanceMeasure->getPerformanceMeasure();
}
