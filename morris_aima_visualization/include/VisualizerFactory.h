/*
* CLASS: VisualizerFactory
* DATE: 06/21/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for generating various Visualizer types.
*/
#ifndef MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H
#define MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H


#include <XmlRpcValue.h>
#include "Visualizer.h"

class VisualizerFactory {
public:
    static Visualizer  *createVisualizer(std::string name, XmlRpc::XmlRpcValue &properties);
};


#endif //MORRIS_AIMA_ENVIRONMENT_ENVIRONMENTFACTORY_H
