/**
* CLASS: SimulatorResultFactory
* DATE: 03/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for building simulator result objects, used in Simulator class.
*/

#ifndef MORRIS_AIMA_SIMULATORRESULTFACTORY_H
#define MORRIS_AIMA_SIMULATORRESULTFACTORY_H
#include <string>
#include "SimulatorResult.h"


class SimulatorResultFactory {
public:
    static SimulatorResult *createSimulatorResult(std::string type, Simulator *sim);
};


#endif //MORRIS_AIMA_SIMULATORRESULTFACTORY_H
