/**
* CLASS: Result
* DATE: June 26, 2016
* AUTHOR: Kyle Morris
* DESCRIPTION:A summary of a project/simulation at a given point.
*/
#ifndef MORRIS_AIMA_SIMULATIONRESULT_H
#define MORRIS_AIMA_SIMULATIONRESULT_H
#include <string>
#include <ros/node_handle.h>

class Result {
private:
    /**
     * The node we're running this on.
     */
    ros::NodeHandle *handle;
public:
    Result(ros::NodeHandle *handle);

    /**
     * Do finishing analysis of data collected at end of a projects runtime.
     * This will handle any calculations that should be done to return the finished/collected results.
     */
    virtual bool summarize() = 0;

    /**
     * Output the simulation results to a file
     * @param outputFile: the file to put results in. These results will be appended.
     * So this is useful if you want to have multiple simulation results all put in the same place.
     * @return bool: true if writing was successful, false otherwise
     */
    virtual bool writeToFile(std::string outputFile) = 0;

    /**
     * Write to a file. A new file will be created with a name specifying the time of simulation.
     * @return bool: true if writing was successful, false otherwise
     */
    virtual bool writeToFile() = 0;

    /**
     * Gather results as a string.
     * @return std::string: The results summarized as a string.
     */
    virtual std::string getResults() = 0;

    ros::NodeHandle *getNodeHandle();

    void setNodeHandle(ros::NodeHandle *handle);

    /**
     * Setup the publishers/services/and other ROS services from within the Result node
     */
    virtual void initialize() = 0;


};
#endif //MORRIS_AIMA_SIMULATIONRESULT_H
