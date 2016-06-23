#include "Environment.h"
#include <ros/service_server.h>
#include <ros/node_handle.h>

long Environment::getAge() {
    return this->age;
}

void Environment::cycle() {
    if(isActive()) {
        ROS_INFO("Cycling environment. AGE %s", getAge());
        act();
        generate();
        updateResults();
        //update environments age
        this->age++;
    }
    publishState();
}

bool Environment::isActive() {
    return this->active;
}

bool Environment::activate() {
    this->active = true;
}

bool Environment::deactivate() {
    this->active = false;
}

ros::NodeHandle *Environment::getNodeHandle() const {
    return this->nodeHandle;
}

void Environment::setNodeHandle(ros::NodeHandle *nodeHandle) {
    this->nodeHandle = nodeHandle;
}
