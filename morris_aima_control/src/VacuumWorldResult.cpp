#include "VacuumWorldResult.h"
#include <string>
#include <sstream>
#include <fstream>
#include <ros/ros.h>

bool VacuumWorldResult::writeToFile(std::string outputFile) {
    std::ofstream output;
    bool result= false;
    const char *str = outputFile.c_str();
    output.open(str, std::ios_base::app);
    if(output.is_open()) {
        output << getResults();
        output.flush();
        output.close();
        result = true;
    }
    return result;
}

bool VacuumWorldResult::writeToFile() {
    /*std::string name;
    std::ostringstream buffer;
    time_t currtime = time(0);
    struct tm * now = localtime( &currtime);
    buffer << (now->tm_year+1900) << "-" << (now->tm_mon +1) << "-" << (now->tm_mday) << "---" << (now->tm_hour) << ":" << (now->tm_min) << ":" << (now->tm_sec) << "\n";
    name =  buffer.str();
    name.erase(name.size()-1);*/
    return writeToFile(outputName);
}

bool VacuumWorldResult::summarize() {
    return true;
}

std::string VacuumWorldResult::getResults() {
    std::ostringstream buffer;
    XmlRpc::XmlRpcValue properties;
    int gridHeight = -1;
    int gridWidth = -1;
    int performanceMeasure = lastFrame.performance_measure;
    int age = lastFrame.age;
    ros::param::get("/morris_aima_environment/config/", properties);
    gridHeight = properties["grid_height"];
    gridWidth = properties["grid_width"];
    time_t currTime = time(0);   // get time now
    struct tm * now = localtime( & currTime );
    buffer << "=====VACUUM WORLD SIMULATION RESULTS=====\n";
    buffer << "\tRan ON:";
    buffer << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' << now->tm_mday;
    buffer << " at " << (now->tm_hour) << ":" << (now->tm_min) << ":" << (now->tm_sec) << "\n";
    buffer << "\tSimulation Cycles: " << age<< "\n";
    buffer << "\tEnvironment Size: [" << gridWidth << "x" << gridHeight << " tiles]\n";
    buffer << "\tPerformance Measure: " << performanceMeasure  << "\n";
    buffer << "\n";
    return buffer.str();
}

void VacuumWorldResult::defaultUpdate_callback(const morris_aima_msgs::TileEnvironmentInfo &msg) {
    if(started) {
        this->lastFrame.age = msg.age;
        this->lastFrame.performance_measure = msg.performance_measure;
        ROS_INFO("environment age %d with performance measure %d!!!", lastFrame.age, lastFrame.performance_measure);
    } else {
        ROS_INFO("Results module not started.");
    }
}

bool VacuumWorldResult::start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    this->started = true;
    return true;
}

bool VacuumWorldResult::stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    this->started = false;
    return true;
}

bool VacuumWorldResult::save_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp) {
    bool result = true;
    if(this->started) {
        ROS_INFO("Please stop the Result module before savin.");
    } else {
        ROS_INFO("Saving results...");
        summarize();
        writeToFile();
    }
    return result;
}

VacuumWorldResult::VacuumWorldResult(ros::NodeHandle *handle) : Result(handle) {
}

void VacuumWorldResult::initialize() {
    this->vacuumWorldSubscriber = getNodeHandle()->subscribe("/morris_aima_environment/tile_environment_info", SUBSCRIPTION_QUEUE_SIZE, &VacuumWorldResult::defaultUpdate_callback, this);
    this->saveService = getNodeHandle()->advertiseService("save", &VacuumWorldResult::save_callback, this);
    this->startService = getNodeHandle()->advertiseService("start", &VacuumWorldResult::start_callback, this);
    this->stopService = getNodeHandle()->advertiseService("stop", &VacuumWorldResult::stop_callback, this);
}
