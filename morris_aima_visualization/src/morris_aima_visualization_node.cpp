#include "ros/ros.h"
#include <QApplication>
#include "std_msgs/String.h"
#include <sstream>
#include <VisualizerFactory.h>
#include <TileVisualizer.h>
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "Visualizer.h"

//BRUTAL hack, saving main arguments so I can pass them to init when thread is made.
/*
static int argc;
static char **argv;

static Visualizer *visualizer;
static int defaultRate = 1; //in hz, how quick we cycle/loop
/**
 * Determine if the given configuration name is a valid environment type to initialize
 * @param worldType: String representing "type" of world we want to init.
 * @param bool: True if valid, false otherwise
 */
/*bool isValidWorldType(std::string worldType) {
    bool result = false;
    if(worldType.compare("vacuum_world")==0) {
        result = true;
    }
    return result;
}

void *spin(void *) {
    ros::Rate loop_rate(defaultRate);
    while(ros::ok()) {
        std::string outputmsg;
        visualizer->render();
        ROS_INFO("too FAST");

        //ros::spinOnce();
        loop_rate.sleep();
    }
    return NULL;
}

*/
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    QApplication application(argc,argv);
    TileVisualizer visualizer(argc, argv);
    visualizer.initialize();
    visualizer.run();
//   ros::init(argc, argv, "morris_aima_visualization_node");

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
 /*   ros::NodeHandle n("morris_aima_visualizer");

    //set up the environment with provided configuration
    std::string worldType;
    ros::param::get("/morris_aima_control/world_type", worldType);
    if(!isValidWorldType(worldType)) {
        ROS_INFO("Invalid type of world specified for configuration: \"%s\"", worldType.c_str());
        return -1;
    }
    XmlRpc::XmlRpcValue properties;
    ros::param::get("/morris_aima_environment/config", properties);

    visualizer = VisualizerFactory::createVisualizer(worldType,properties);

    if(visualizer==NULL) {
        ROS_INFO("ERROR: Unable to build visualizer.");
        return -1;
    }

    visualizer->setNodeHandle(&n);
    ROS_INFO("Configuration complete. Initializing visualizer.");
    visualizer->initialize();
    ROS_INFO("Visualizer successfully Initialized!");
    visualizer->render();
    //services for activating/deactivating and checking activation of environment.
    //run main loop, so long as the ros node is up and running.
    pthread_t rosspin;
    pthread_create(&rosspin, NULL, spin, NULL);
    */

    application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));
    int result = application.exec();
    return result;

}
