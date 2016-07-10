/**
* CLASS: VacuumWorldResult
* DATE: July 5, 2016
* AUTHOR: Kyle Morris
* DESCRIPTION: Results of a vacuum world simulation.
*/
#ifndef MORRIS_AIMA_CONTROL_VACUUMWORLDRESULT_H
#define MORRIS_AIMA_CONTROL_VACUUMWORLDRESULT_H
#include "Result.h"
#include "morris_aima_msgs/TileEnvironmentInfo.h"
#include "std_srvs/Empty.h"

class VacuumWorldResult : public Result {
private:
    /**
     * The name of the file we will append results to.
     */
    std::string outputName = "vacuum_world_results";

    /**
     * We just track the last recorded info here.
     */
    morris_aima_msgs::TileEnvironmentInfo lastFrame;
    /**
     * Whether or not the result module is recording/tracking the environment.
     */
    bool started = false;
    /*
     * At most, how many messages to keep enqueued when subscribing.
     */
    const int SUBSCRIPTION_QUEUE_SIZE = 1000;

    /**
     * Tracking the performance measure, we don't need to record all of them as they change.
     */
    int performanceMeasure = 0;

    /**
     * How old the environment currently is.
     */
    int environmentAge = 0;

    ros::Subscriber vacuumWorldSubscriber;

    //services to start, stop, and save results we're recording.
    ros::ServiceServer startService;
    ros::ServiceServer stopService;
    ros::ServiceServer saveService;
public:
    VacuumWorldResult(ros::NodeHandle *handle);

    virtual bool writeToFile(std::string outputFile);

    virtual bool writeToFile();

    virtual bool summarize();

    virtual std::string getResults();

    virtual void initialize();

    void defaultUpdate_callback(const morris_aima_msgs::TileEnvironmentInfo &msg);
    bool start_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool stop_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    bool save_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
};


#endif //MORRIS_AIMA_CONTROL_VACUUMWORLDRESULT_H
