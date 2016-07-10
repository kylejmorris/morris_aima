#include "Result.h"
#include <fstream>


Result::Result(ros::NodeHandle *handle) {
    this->handle = handle;
}


bool Result::writeToFile() {
    //TODO finish this, can't while on a plane, need to have docs/etc.
/*    std::ofstream output;
    std::string fileName = "VacuumWorld_sim_output_TIMESTAMP.out";
    bool result = false;
    output.open(fileName);
*/
}

ros::NodeHandle *Result::getNodeHandle() {
    return this->handle;
}

bool Result::writeToFile(std::string outputFile) {
    std::ofstream output;
    bool result = false; //whether or not writing to file worked
    /*output.open(outputFile);
    if(output.is_open()) {
        output << getResults();
        output.flush();
        output.close();
        result = true;
    }*/
    return result;
}

std::string Result::getResults() {
    return "this simulation has no results.";
}

void Result::setNodeHandle(ros::NodeHandle *handle) {
    this->handle = handle;
}


