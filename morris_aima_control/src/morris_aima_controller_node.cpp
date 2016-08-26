#include "ros/ros.h"
#include "ros/node_handle.h"
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include <Result.h>
#include <ResultFactory.h>
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "MainController.h"

static int defaultRate = 1; //in hz, how quick we cycle/loop
/**
 * Determine if the given configuration name is a valid environment type to initialize
 * @param worldType: String representing "type" of world we want to init.
 * @param bool: True if valid, false otherwise
 */
bool isValidWorldType(std::string worldType) {
    bool result = false;
    if(worldType.compare("vacuum_world")==0) {
        result = true;
    }
    return result;
}

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    int cycleRate = defaultRate; //how quick we'll loop
    ros::init(argc, argv, "morris_aima_controller_node");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n("morris_aima_controller");

    //set up the environment with provided configuration
    std::string worldType;
    ros::param::get("/morris_aima_control/world_type", worldType);
    if(!isValidWorldType(worldType)) {
        ROS_INFO("Invalid type of world specified for configuration: \"%s\"", worldType.c_str());
        return -1;
    }
    XmlRpc::XmlRpcValue properties;

    ros::Rate loop_rate(cycleRate);

    MainController *controller = new MainController(&n);

    if(controller==NULL) {
        ROS_INFO("ERROR: Unable to build controller object.");
        return -1;
    }

    ROS_INFO("Configuration complete. Initializing controller module.");
    controller->initialize();
    ROS_INFO("Controller successfully Initialized!");

    //services for activating/deactivating and checking activation of environment.
    //run main loop, so long as the ros node is up and running.
    while(ros::ok()) {
        std::string outputmsg;
        ROS_INFO("Running controller...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
