/**
* CLASS: MandCSimulationResult
* DATE: 10/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Dummy class, just here to fit framework. No real file output
 * needed for the simulation unless I feel like it.
*/

#ifndef MORRIS_AIMA_MANDCSIMULATIONRESULT_H
#define MORRIS_AIMA_MANDCSIMULATIONRESULT_H


#include <SimulatorResult.h>

class MandCSimulationResult : public SimulatorResult {
public:
    virtual std::string getResults() override;
};


#endif //MORRIS_AIMA_MANDCSIMULATIONRESULT_H
