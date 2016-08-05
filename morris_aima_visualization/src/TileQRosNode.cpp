#include <TileQRosNode.h>
#include <ros/init.h>
#include <ros/rate.h>
#include <ros/node_handle.h>
#include <ros/ros.h>

bool TileQRosNode::enableUpdating_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    Q_EMIT enableUpdating();
    return true;
}

bool TileQRosNode::setParameters_callback(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response) {
    XmlRpc::XmlRpcValue properties;
    int width = -1;
    int height = -1;
    ros::param::get("/morris_aima_environment/config",properties);
    width = properties["grid_width"];
    height = properties["grid_height"];
    Q_EMIT setParameters(width,height);
}

void TileQRosNode::reset_callback() {

}

void TileQRosNode::update_callback(const morris_aima_msgs::TileEnvironmentInfo &msg) {
    ROS_INFO("trying to updatE!!!!!");
    Q_EMIT update(msg);
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
    ros::NodeHandle handle("morris_aima_visualizer");

    set_parameters_service = handle.advertiseService("set_parameters",&TileQRosNode::setParameters_callback, this);
    enable_update_service = handle.advertiseService("start",&TileQRosNode::enableUpdating_callback, this);
    update_subscriber = handle.subscribe("/morris_aima_environment/tile_environment_info",1000,&TileQRosNode::update_callback,this);

    start_time = ros::Time::now();

    //starting a QThread for ros
    start();
}

TileQRosNode::TileQRosNode(int argc, char **argv) {
    this->argc = argc;
    this->argv = argv;
}
