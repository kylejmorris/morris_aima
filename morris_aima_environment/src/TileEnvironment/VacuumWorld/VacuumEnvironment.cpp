#include <VacuumPercept.h>
#include <DirtEntity.h>
#include <ros/package.h>
#include <json/json.h>
#include <ros/node_handle.h>
#include <std_srvs/Empty.h>
#include "SimpleVacuumAction.h"
#include "VacuumEnvironment.h"
#include "SimpleReflexVacuumAgent.h"
#include "SimpleVacuumAction.h"
#include "RandomVacuumAction.h"

void VacuumEnvironment::reset() {
    if(isLoaded()) {
        TileEnvironment::reset();
        delete this->state;
        if(this->vacuumLocation!=NULL) {
            delete this->vacuumLocation;
        }
        delete this->vacuum;
        delete this->performanceMeasure;
        this->performanceMeasure = new VacuumWorldPerformanceMeasure;
        ROS_INFO("Environment has been reset!");
    } else {
        ROS_INFO("Nothing to reset!");
    }
}

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

void VacuumEnvironment::publish() {
    TileEnvironment::publish();
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
    TileEnvironment::load(fileName);
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

bool VacuumEnvironment::load_callback(morris_aima_msgs::Load::Request &req, morris_aima_msgs::Load::Response &resp) {
    if(isActive()) {
        ROS_INFO("NOTICE: Please deactivate the environment before loading a new one.");
    } else {
        if(isLoaded()) {
            ROS_INFO("First resetting environment before load.");
            this->reset();
        }
        std::string mapname = req.map_name;
        this->load(mapname);
        //By keeping a VacuumWorldState instead of using TileEnvironmentState, we don't have to keep downcasting for subclass methods.
        this->state = static_cast<VacuumEnvironmentState *>(TileEnvironment::readState()); //since it was figured out in parent, must bring it here. as a VacuumWorld State
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

bool VacuumEnvironment::activate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    if(isLoaded()) {
        activate();
    } else {
        ROS_INFO("error: Environment has not yet been loaded!");
    }
}

bool VacuumEnvironment::deactivate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    deactivate();
}

bool VacuumEnvironment::reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    if(isActive()) {
        ROS_INFO("First stopping environment...");
        deactivate();
    }
    reset();
    return true;
}

void VacuumEnvironment::initialize() {
    TileEnvironment::initialize();
    activateService = getNodeHandle()->advertiseService("start", &VacuumEnvironment::activate_callback, this);
    deactivateService = getNodeHandle()->advertiseService("stop", &VacuumEnvironment::deactivate_callback, this);
    loadService = getNodeHandle()->advertiseService("load", &VacuumEnvironment::load_callback, this);
    resetService = getNodeHandle()->advertiseService("reset", &VacuumEnvironment::reset_callback, this);
}

double VacuumEnvironment::getPerformanceMeasure() {
    return this->performanceMeasure->getPerformanceMeasure();
}
