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
    /**
     * Create a simulator.
     * @param type: The type of simulator, this will determine how the factory builds it.
     * @param name: The name of frame/visualizer that displays the simulation.
     * @param mapFile: Path to location of map to load into environmnet
     * @param  cycleTime: how long a cycle takes
     */
    static Simulator *createSimulator(std::string type, std::string name, std::string mapFile, long cycleTime);
};


#endif //MORRIS_AIMA_SIMULATORFACTORY_H
