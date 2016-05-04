/**
* CLASS: SimulatorFactory
* DATE: 03/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for making simulators, based on project selected.
*/
#ifndef MORRIS_AIMA_SIMULATORFACTORY_H
#define MORRIS_AIMA_SIMULATORFACTORY_H

#include <Simulator.h>

class SimulatorFactory {
public:
    static Simulator *createSimulator(std::string name);
};


#endif //MORRIS_AIMA_SIMULATORFACTORY_H
