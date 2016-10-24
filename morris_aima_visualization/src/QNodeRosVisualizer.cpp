#include <ros/init.h>
#include <QThread>
#include <ros/rate.h>
#include <QtCore/qobjectdefs.h>
#include <VisualizerFactory.h>
#include <QtGui/QtGui>
#include <QtCore/QCoreApplication>
#include "QNodeRosVisualizer.h"

static int defaultRate = 1; //in hz, how quick we cycle/loop

/**
 * Determine if the given configuration name is a valid environment type to initialize
 * @param worldType: String representing "type" of world we want to init.
 * @param bool: True if valid, false otherwise
 */
bool QNodeRosVisualizer::isValidWorldType(std::string worldType) {
    bool result = false;
    if(worldType.compare("vacuum_world")==0) {
        result = true;
    }
    return result;
}

void QNodeRosVisualizer::initialize(int argc, char **argv) {
    ros::init(argc, argv, "morris_aima_visualization_node");
    ros::start();

    /**
     * NodeHandle is the main access point to communications with the ROS system.
     * The first NodeHandle constructed will fully initialize this node, and the last
     * NodeHandle destructed will close down the node.
     */
    ros::NodeHandle n("morris_aima_visualizer");

    //set up the environment with provided configuration
    std::string worldType;
    ros::param::get("/morris_aima_control/world_type", worldType);
    if(!isValidWorldType(worldType)) {
        ROS_INFO("Invalid type of world specified for configuration: \"%s\"", worldType.c_str());
    }
    XmlRpc::XmlRpcValue properties;
    ros::param::get("/morris_aima_environment/config", properties);

    visualizer = VisualizerFactory::createVisualizer(worldType,properties);

    if(visualizer==NULL) {
        ROS_INFO("ERROR: Unable to build visualizer.");
    }

    visualizer->setNodeHandle(&n);
    ROS_INFO("Configuration complete. Initializing visualizer.");
    visualizer->initialize();
    ROS_INFO("Visualizer successfully Initialized!");
}

void QNodeRosVisualizer::render() {
    this->visualizer->render();
}

void QNodeRosVisualizer::run() {
    int cycleRate = defaultRate; //how quick we'll loop
    ros::Rate loop_rate(cycleRate);
    std::string outputmsg;
    ROS_INFO("RUNNING ROS LOOP");
    ros::spinOnce();
    this->visualizer->render();
}
