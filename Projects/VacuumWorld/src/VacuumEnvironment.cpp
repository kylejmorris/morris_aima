#include <VacuumPercept.h>
#include <DirtEntity.h>
#include <json/json.h>
#include "SimpleVacuumAction.h"
#include "VacuumEnvironment.h"
#include "SimpleReflexVacuumAgent.h"
#include "SimpleVacuumAction.h"
#include "RandomVacuumAction.h"

VacuumEnvironment::VacuumEnvironment() : TileEnvironment() {
    this->performanceMeasure = new VacuumWorldPerformanceMeasure;
}

void VacuumEnvironment::act() {
    this->vacuum = this->state->findVacuum();
    this->vacuumLocation = this->state->getLocationOf(this->vacuum->getId());
    if(this->vacuum!=NULL) { //did we find vacuum?
        VacuumPercept *forVacuum = new VacuumPercept(vacuumLocation, this->state->isDirty(vacuumLocation->getX(), vacuumLocation->getY()));

        //act depending on the type of vacuum, we will have different actions possible.
        if(this->vacuum->getType().compare("RandomReflexVacuumAgent")==0) {
            RandomVacuumAction *action = (RandomVacuumAction *)(this->vacuum->think(forVacuum));
            randomVacuumAct(action);
            delete action;
        } else { //if simple agent
            SimpleVacuumAction *action = (SimpleVacuumAction *)(this->vacuum->think(forVacuum));
            simpleVacuumAct(action);
            delete action;
        }
        delete forVacuum;
    }
}

void VacuumEnvironment::generate() {

}

VacuumEnvironment::~VacuumEnvironment() {
}

void VacuumEnvironment::updateResults() {
    this->performanceMeasure->update(readState());
}

std::string VacuumEnvironment::outputToJson() {
    Json::Value outputRoot;
    std::string result;
    //general environment info
    outputRoot["Environment"]["age"] = (int) getAge();
    outputRoot["Environment"]["size"]["x"] = state->getWidth();
    outputRoot["Environment"]["size"]["y"] = state->getHeight();

    //handling all entity info in the environment
    outputRoot["Environment"]["entities"] = Json::arrayValue;
    for (int row = 0; row < this->state->getHeight(); row++) {
        for (int col = 0; col < state->getWidth(); col++) {
            TileLocation location(col, row);
            std::vector<Entity *> onTile = state->getEntitiesAt(&location);
            for (auto current : onTile) {
                Json::Value currentEntity; //current entity on a tile we're making info for
                currentEntity["type"] = current->getType();
                currentEntity["id"] = current->getId();
                currentEntity["location"]["x"] = col;
                currentEntity["location"]["y"] = row;

                outputRoot["Environment"]["entities"].append(currentEntity);
            }
        }
    }

    outputRoot["debug"]["performance_measure"] = getPerformanceMeasure();
    result = outputRoot.toStyledString();

    return result;
}

EnvironmentState *VacuumEnvironment::readState() {
    return this->state;
}

void VacuumEnvironment::loadEnvironment(string fileName) {
    TileEnvironment::loadEnvironment(fileName);
    this->state = dynamic_cast<VacuumEnvironmentState *>(TileEnvironment::readState());
}

void VacuumEnvironment::simpleVacuumAct(SimpleVacuumAction *action) {
    //respond to the action made by agent, updating environment state
    if(action->choice == SimpleVacuumAction::LEFT) {
        if(!state->hasWall(this->vacuumLocation->getX()-1, this->vacuumLocation->getY())) {
            state->moveVacuum(this->vacuumLocation->getX()-1, this->vacuumLocation->getY());
        }
    } else if(action->choice == SimpleVacuumAction::RIGHT) {
        if(!state->hasWall(this->vacuumLocation->getX()+1, this->vacuumLocation->getY())) {
            state->moveVacuum(this->vacuumLocation->getX() + 1, this->vacuumLocation->getY());
        }
    } else if(action->choice == SimpleVacuumAction::SUCK) {
        state->cleanTile();
    }
}

void VacuumEnvironment::randomVacuumAct(RandomVacuumAction *action) {
    if(action!=NULL) {
//respond to the action made by agent, updating environment state
        if (action->choice == RandomVacuumAction::LEFT) {
            if (this->vacuumLocation->getX()-1 >=0 && !state->hasWall(this->vacuumLocation->getX() - 1, this->vacuumLocation->getY())) {
                state->moveVacuum(this->vacuumLocation->getX() - 1, this->vacuumLocation->getY());
            }
        } else if (action->choice == RandomVacuumAction::RIGHT) {
            if (this->vacuumLocation->getX()+1 < state->getWidth() && !state->hasWall(this->vacuumLocation->getX() + 1, this->vacuumLocation->getY())) {
                state->moveVacuum(this->vacuumLocation->getX() + 1, this->vacuumLocation->getY());
            }
        } else if (action->choice == RandomVacuumAction::UP) {
            if (this->vacuumLocation->getY()-1>=0 && !state->hasWall(this->vacuumLocation->getX(), this->vacuumLocation->getY() - 1)) {
                state->moveVacuum(this->vacuumLocation->getX(), this->vacuumLocation->getY() - 1);
            }
        } else if (action->choice == RandomVacuumAction::DOWN) {
            if (this->vacuumLocation->getY()+1<state->getHeight() && !state->hasWall(this->vacuumLocation->getX(), this->vacuumLocation->getY() + 1)) {
                state->moveVacuum(this->vacuumLocation->getX(), this->vacuumLocation->getY() + 1);
            }
        } else if (action->choice == RandomVacuumAction::SUCK) {
            state->cleanTile();
        }
    }
}

double VacuumEnvironment::getPerformanceMeasure() {
    return this->performanceMeasure->getPerformanceMeasure();
}
