#include <TileQRosNode.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

void TileQRosNode::enableUpdating_callback() {
}

void TileQRosNode::setParameters_callback(int width, int height, std::string name) {

}

void TileQRosNode::reset_callback() {

}

void TileQRosNode::update_callback(morris_aima_msgs::TileEnvironmentInfo &msg) {

}


void TileQRosNode::run() {
    ros::Rate loop_rate(1);

    while ( ros::ok() )
    {
        ROS_INFO("Ros is running...");
        ros::spinOnce();
        loop_rate.sleep();
    }

    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

bool TileQRosNode::initialize() {
    ros::init(argc, argv, "morris_aima_visualizer");

    if ( ! ros::master::check() ) {
        return false;
    }

    //explicitly needed as nodehandle is going out of scope.
    ros::start();
    ros::NodeHandle handle;

    start_time = ros::Time::now();

    //starting a QThread for ros
    start();
}

TileQRosNode::TileQRosNode(int argc, char **argv) {
    this->argc = argc;
    this->argv = argv;
}
