#include "Environment.h"
#include <ros/service_server.h>
#include <ros/node_handle.h>

long Environment::getAge() {
    return this->age;
}

void Environment::cycle() {
    if(isActive()) {
        int age = getAge();
        ROS_INFO("Cycling environment. AGE %d", age);
        act();
        generate();
        updateResults();
        //update environments age
        this->age++;
    }
    publish();
}

bool Environment::isActive() {
    return this->active;
}

bool Environment::activate() {
    if(isActive()) {
        ROS_INFO("NOTICE: Environment is already activated.");
    } else {
        this->active = true;
    }
}

bool Environment::deactivate() {
    if(!isActive()) {
        ROS_INFO("NOTICE: Environment is already deactivated.");
    } else {
        this->active = false;
    }
}

void Environment::reset() {
    this->age = 0; //just resetting the age
}

ros::NodeHandle *Environment::getNodeHandle() const {
    return this->nodeHandle;
}

void Environment::setNodeHandle(ros::NodeHandle *nodeHandle) {
    this->nodeHandle = nodeHandle;
}
