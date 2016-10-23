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
    std_srvs::Empty start;
    ros::service::call("/morris_aima_environment/start", start);
    ros::service::call("/morris_aima_visualizer/start", start);
    return true;
}

bool MainController::stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    std_srvs::Empty stop;
    ros::service::call("/morris_aima_environment/stop", stop);
    return true;
}

bool MainController::reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    std_srvs::Empty reset;
    ROS_INFO("Resetting!");
    ros::service::call("/morris_aima_environment/reset", reset);
    return false;
}

bool MainController::save_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    ROS_INFO("Saving is not yet implemented.");
    return false;
}

ros::NodeHandle *MainController::getNodeHandle() {
    return this->handle;
}

bool MainController::load_callback(morris_aima_msgs::Load::Request &req, morris_aima_msgs::Load::Response &resp) {
    morris_aima_msgs::Load load_req;
    std_srvs::Empty set_parameters;
    load_req.request.map_name = req.map_name;
    ros::service::call("/morris_aima_environment/load",load_req);
    ROS_INFO("Load service successful!.");
    ros::service::call("/morris_aima_visualizer/set_parameters", set_parameters);
    return true;
}
