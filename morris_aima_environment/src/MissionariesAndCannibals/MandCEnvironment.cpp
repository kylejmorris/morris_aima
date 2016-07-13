#include <json/json.h>
#include "Environment.h"
#include "MandCEnvironment.h"
#include <iostream>
#include "MandCEnvironmentState.h"
#include <vector>
#include "StateNode.h"
#include "MandCAction.h"

void MandCEnvironment::loadEnvironment(string fileName) {
    StateNode *initialNode = NULL;
    MandCEnvironmentState *initialState = new MandCEnvironmentState;
    initialState->setCannibalsLeft(1);
    initialState->setCannibalsRight(2);
    initialState->setMissionariesLeft(3);
    initialState->setRiverCrossed(false);
    initialState->setMissionariesRight(0);
    initialNode = new StateNode(0, initialState, NULL, NULL);
    expandNode(initialNode);
    this->goalNode = NULL;
    this->initialState = new MandCEnvironmentState; //automatically does initial setup
    this->goalState = new MandCEnvironmentState;
    this->goalState->setCannibalsLeft(0);
    this->goalState->setCannibalsRight(3);
    this->goalState->setMissionariesLeft(0);
    this->goalState->setMissionariesRight(3);
    this->goalState->setRiverCrossed(true);

    initialNode = new StateNode(0, this->initialState, NULL, NULL);
    //in case the initial node has our goal state, just stop the search before we start.
    if(initialNode->getState()->compareTo(goalState)==0) {
        goalNode = new StateNode(0, initialState, NULL, NULL);
    } else {
        addToFrontier(initialNode);
    }
}

EnvironmentState *MandCEnvironment::readState() {
    return NULL;
}

bool MandCEnvironment::add(Entity *e, Location *place) {
    return false;
}

Entity *MandCEnvironment::remove(int id) {
    return NULL;
}

bool MandCEnvironment::exists(int id) {
    return false;
}

std::vector<Entity *> MandCEnvironment::getEntities() {
    return std::vector<Entity *>();
}

Json::Value MandCEnvironment::outputStateToJson(MandCEnvironmentState *state) {
    Json::Value outputNode;

    outputNode["cannibals_left"] = state->getCannibalsLeft();
    outputNode["cannibals_right"] = state->getCannibalsRight();
    outputNode["missionaries_left"] = state->getMissionariesLeft();
    outputNode["missionaries_right"] = state->getMissionariesRight();
    outputNode["river_crossed"] = state->isRiverCrossed();

    return outputNode;
}

std::string MandCEnvironment::outputToJson() {
    Json::Value outputRoot;
    std::string result;
    //general environment info
    outputRoot["Environment"]["age"] = (int) getAge();
    outputRoot["Environment"]["type"] = "MissionariesAndCannibals";
    if(goalNode!=NULL) {
        outputRoot["Environment"]["found"] = true;
        outputRoot["Environment"]["solution_path"] = Json::arrayValue;
        StateNode *currentPathStep = goalNode;
        int count = 0;
        while (currentPathStep != NULL) {
            Json::Value temp;
            temp["id"] = count;
            temp["state"] = outputStateToJson(currentPathStep->getState());
            outputRoot["Environment"]["solution_path"].append(temp);
            currentPathStep = currentPathStep->getParent();
            count++;
        }
    } else {
        outputRoot["Environment"]["found"] = false;
        outputRoot["Environment"]["frontier"] = Json::arrayValue;
        for(std::vector<StateNode *>::iterator current = frontier.begin(); current<frontier.end(); current++) {
            outputRoot["Environment"]["frontier"].append(outputStateToJson((*current)->getState()));
        }
        outputRoot["Environment"]["explored"] = Json::arrayValue;
        for(std::vector<StateNode *>::iterator current = explored.begin(); current<explored.end(); current++) {
            outputRoot["Environment"]["explored"].append(outputStateToJson((*current)->getState()));
        }
    }

    result = outputRoot.toStyledString();
    return result;
}

double MandCEnvironment::getPerformanceMeasure() {
    return 0;
}

void MandCEnvironment::act() {
    if (!goalFound()) {
        if (frontier.empty()) {
            std::cout << "ERROR NO POSSIBLE SOLUTION " << std::endl;
        } else {
            //remove from frontier
            StateNode *popped = frontier.at(0);
            frontier.erase(frontier.begin());
            //mark as explored
            addToExplored(popped);

            //expanding node
            std::vector<StateNode *> expanded = expandNode(popped);

            for (StateNode *curr : expanded) {
                if (!inFrontier(curr) && !inExploredSet(curr)) {
                    if (curr->getState()->compareTo(goalState) == 0) {
                        this->goalNode = curr;
                    } else {
                        addToFrontier(curr);
                    }
                }
            }
        }
    }
}

void MandCEnvironment::generate() {
}

void MandCEnvironment::updateResults() {
}

void MandCEnvironment::addToFrontier(StateNode *target) {
    frontier.push_back(target);
}

void MandCEnvironment::addToExplored(StateNode *target) {
    explored.push_back(target);
}

bool MandCEnvironment::goalFound() {
    bool result = true;

    if(goalNode==NULL) {
        result = false;
    }

    return result;
}

std::vector<StateNode *> MandCEnvironment::expandNode(StateNode *target) {
    std::vector<StateNode *> expanded;
    //running through each possible action
    MandCEnvironmentState *expandedState = target->getState()->ship(1,0);
    StateNode *curr = new StateNode(target->getPathCost()+1,expandedState,target, new MandCAction(1,0));
    expanded.push_back(curr);

    expandedState = target->getState()->ship(0,1);
    curr = new StateNode(target->getPathCost()+1, expandedState, target, new MandCAction(0,1));
    expanded.push_back(curr);

    expandedState = target->getState()->ship(1,1);
    curr = new StateNode(target->getPathCost()+1, expandedState, target, new MandCAction(1,1));
    expanded.push_back(curr);

    expandedState = target->getState()->ship(2,0);
    curr = new StateNode(target->getPathCost()+1, expandedState, target, new MandCAction(2,0));
    expanded.push_back(curr);

    expandedState = target->getState()->ship(0,2);
    curr = new StateNode(target->getPathCost()+1, expandedState, target, new MandCAction(0,2));
    expanded.push_back(curr);

    for (std::vector<StateNode *>::iterator it = expanded.begin(); it < expanded.end(); it++) {
        //delete all the invalid nodes that were generated
        StateNode *node = *it;
        MandCEnvironmentState *state = node->getState();
        if (state == NULL) { //this state must be invalid
            it = expanded.erase(it);
            it--;
        }
    }

    return expanded;
}

bool MandCEnvironment::inFrontier(StateNode *node) {
    bool result = false;
    for (StateNode *curr : frontier) {
        if (curr->compareTo(node) == 0) {
            result = true;
        }
    }

    return result;
}

bool MandCEnvironment::inExploredSet(StateNode *node) {
    bool result = false;

    for (StateNode *curr : explored) {
        if (curr->compareTo(node) == 0) {
            result = true;
        }
    }

    return result;
}


bool MandCEnvironment::activate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    this->activate();
}

bool MandCEnvironment::deactivate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    this->deactivate();
}

bool MandCEnvironment::reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    if(isActive()) {
        ROS_INFO("First stopping environment...");
        this->deactivate();
    }
    this->reset();
    return true;
}

void MandCEnvironment::initialize() {
    activateService = getNodeHandle()->advertiseService("start", &MandCEnvironment::activate_callback, this);
    deactivateService = getNodeHandle()->advertiseService("stop", &MandCEnvironment::deactivate_callback, this);
    resetService = getNodeHandle()->advertiseService("reset", &MandCEnvironment::reset_callback, this);
}

void MandCEnvironment::publish() {

}
