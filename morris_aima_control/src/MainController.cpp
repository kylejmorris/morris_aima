#include <ros/node_handle.h>
#include <morris_aima_msgs/Load.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/Empty.h>
#include <ros/ros.h>
#include "MainController.h"

MainController::MainController(ros::NodeHandle *handle) {
    this->handle = handle;
}

void MainController::initialize() {
    this->save_service = getNodeHandle()->advertiseService("save", &MainController::save_callback, this);
    this->start_service = getNodeHandle()->advertiseService("start", &MainController::start_callback, this);
    this->stop_service = getNodeHandle()->advertiseService("stop", &MainController::stop_callback, this);
    this->load_service = getNodeHandle()->advertiseService("load", &MainController::load_callback, this);
}

bool MainController::start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    return false;
}

bool MainController::stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    return false;
}

bool MainController::save_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    return false;
}

ros::NodeHandle *MainController::getNodeHandle() {
    return this->handle;
}

bool MainController::load_callback(morris_aima_msgs::Load::Request &req, morris_aima_msgs::Load::Response &resp) {
    morris_aima_msgs::Load load_req;
    load_req.request.map_name = "test";
    ros::service::call("/morris_aima_environment/load",load_req);
    ROS_INFO("Called!");
    return true;
}
