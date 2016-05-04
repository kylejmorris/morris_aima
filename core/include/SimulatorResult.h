/**
* CLASS: SimulationResult
* DATE: 01/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION:A summary of a Simulation at a given point. Basically just a class to grab info from a Simulation object and display it/perform operations on it.
*/
#ifndef MORRIS_AIMA_SIMULATIONRESULT_H
#define MORRIS_AIMA_SIMULATIONRESULT_H
#include <string>

class Simulator;
class Environment;
class Visualizer;

class SimulatorResult {
private:
    /**
     * The simulation we are going to print results of.
     */
    Simulator *simulation;
public:
    SimulatorResult(Simulator *target);

    Simulator *getSimulator();

    Environment *getEnvironment();

    Visualizer *getVisualizer();
    /**
     * Return results summary as a string.
     */
    virtual std::string getResults() = 0;
};

#endif //MORRIS_AIMA_SIMULATIONRESULT_H
