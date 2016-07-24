#include <ros/node_handle.h>
#include "Visualizer.h"

ros::NodeHandle *Visualizer::getNodeHandle() {
    return this->handle;
}

void Visualizer::setNodeHandle(ros::NodeHandle *handle) {
    this->handle = handle;
}

