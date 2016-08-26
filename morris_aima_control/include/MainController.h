/*
 * class: MainController
 * author: Kyle Morris
 * Description: connects all the components of a project
 */
#ifndef MORRIS_AIMA_CONTROL_MAINCONTROLLER_H
#define MORRIS_AIMA_CONTROL_MAINCONTROLLER_H
#include <morris_aima_msgs/Load.h>
#include "Result.h"
#include "morris_aima_msgs/TileEnvironmentInfo.h"
#include "std_srvs/Empty.h"
#include "std_srvs/Trigger.h"

class MainController {
private:
    ros::NodeHandle *handle;

    //services we can call on MainController
    ros::ServiceServer start_service;
    ros::ServiceServer stop_service;
    ros::ServiceServer save_service;
    ros::ServiceServer load_service;

    //clients to call various modules from controller
    ros::ServiceClient environment_load_client;
    ros::ServiceClient visualizer_load_client;
public:
    MainController(ros::NodeHandle *handle);

    virtual void initialize();

    ros::NodeHandle *getNodeHandle();

    bool start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool save_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool load_callback(morris_aima_msgs::Load::Request &req, morris_aima_msgs::Load::Response &resp);
};
#endif //MORRIS_AIMA_CONTROL_MAINCONTROLLER_H
