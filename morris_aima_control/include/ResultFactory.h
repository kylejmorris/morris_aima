/**
* CLASS: ResultFactory
* DATE: July 5, 2016
* AUTHOR: Kyle Morris
* DESCRIPTION: Select which Result manager to use.
*/
#ifndef MORRIS_AIMA_CONTROL_RESULTFACTORY_H
#define MORRIS_AIMA_CONTROL_RESULTFACTORY_H

#include <Result.h>

class ResultFactory {
public:
    static Result* createResult(std::string type, ros::NodeHandle *handle, XmlRpc::XmlRpcValue properties);
};

#endif //MORRIS_AIMA_CONTROL_RESULTFACTORY_H
