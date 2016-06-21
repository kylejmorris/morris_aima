/*
* CLASS: EnvironmentFactory
* DATE: 06/21/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for generating various Environment types.
*/
#ifndef MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H
#define MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H


#include <XmlRpcValue.h>
#include "Environment.h"

class EnvironmentFactory {
public:
    static Environment  *createEnvironment(std::string name, XmlRpc::XmlRpcValue &properties);
};


#endif //MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H
