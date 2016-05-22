/**
* CLASS: VacuumWorldSimulationResult
* DATE: 02/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Record of results of a VacuumWorld simulation.
*/
#ifndef MORRIS_AIMA_VACUUMWORLDSIMULATIONRESULT_H
#define MORRIS_AIMA_VACUUMWORLDSIMULATIONRESULT_H


#include "SimulatorResult.h"
class VacuumWorldSimulationResult : public SimulatorResult {

public:
    VacuumWorldSimulationResult(Simulator *sim);
    virtual std::string getResults() override;

    virtual bool writeToFile(std::string outputFile) override;
};


#endif //MORRIS_AIMA_VACUUMWORLDSIMULATIONRESULT_H
