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

class EnvironmentStateFactory {
public:

    static EnvironmentState *createEnvironmentState(std::string type, Json::Value properties);
};


#endif //MORRIS_AIMA_ENVIRONMENTSTATEFACTORY_H
