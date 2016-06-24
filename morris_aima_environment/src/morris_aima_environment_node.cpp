#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <Environment.h>
#include <EnvironmentFactory.h>
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"

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
    ros::init(argc, argv, "morris_aima_environment_node");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n("morris_aima_environment");

    //set up the environment with provided configuration
    std::string worldType;
    ros::param::get("/morris_aima_control/world_type", worldType);
    if(!isValidWorldType(worldType)) {
        ROS_INFO("Invalid type of world specified for configuration: \"%s\"", worldType.c_str());
        return -1;
    }
    XmlRpc::XmlRpcValue properties;

    //checking if frequency was configured
    ros::param::get("/morris_aima_environment/config", properties);
    if(properties.hasMember("cycle_freq")) {
        cycleRate = properties["cycle_freq"];
    }

    ros::Rate loop_rate(cycleRate);

    Environment *environment = EnvironmentFactory::createEnvironment(worldType,properties);

    if(environment==NULL) {
        ROS_INFO("ERROR: Unable to build environment.");
        return -1;
    }

    environment->setNodeHandle(&n);
    ROS_INFO("Configuration complete. Initializing environment.");
    environment->initialize();
    ROS_INFO("Environment successfully Initialized!");
    //services for activating/deactivating and checking activation of environment.
    //run main loop, so long as the ros node is up and running.
    while(ros::ok()) {
        std::string outputmsg;

        if(environment->isActive()) {
            outputmsg = "active";
        } else {
            outputmsg = "inactive";
        }

        environment->cycle();
        ROS_INFO("%s", outputmsg.c_str());
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
