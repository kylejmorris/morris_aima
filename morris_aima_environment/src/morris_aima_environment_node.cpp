#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <Environment.h>
#include <EnvironmentFactory.h>

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    ros::init(argc, argv, "morris_aima_environment_node");
    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n("morris_aima_environment");

    //set of the environment with provided configuration
    std::string worldType;
    ros::param::get("/morris_aima_control/world_type", worldType);
    XmlRpc::XmlRpcValue properties;
    ros::param::get("config", properties);
    Environment *environment = EnvironmentFactory::createEnvironment(worldType,properties);

    //run main loop, so long as the ros node is up and running.
    while(ros::ok()) {
    }

    ros::spin();
    return 0;
}
