/**
* CLASS: EnvironmentStateFactory
* DATE: 30/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Associate correct environment state with a given environment
*/

#ifndef MORRIS_AIMA_ENVIRONMENTSTATEFACTORY_H
#define MORRIS_AIMA_ENVIRONMENTSTATEFACTORY_H


#include <EnvironmentState.h>
#include <string>
#include <XmlRpcValue.h>

class EnvironmentStateFactory {
public:
    static EnvironmentState *createEnvironmentState(std::string type, Json::Value properties);
    static EnvironmentState *createEnvironmentState(std::string type, XmlRpc::XmlRpcValue stateProperties);
};


#endif //MORRIS_AIMA_ENVIRONMENTSTATEFACTORY_H
