#include <sstream>
#include <VacuumEnvironment.h>
#include "VacuumWorldSimulationResult.h"
#include "Simulator.h"


std::string VacuumWorldSimulationResult::getResults() {
    std::ostringstream buffer;
    Simulator *s = getSimulator();
    VacuumEnvironmentState *state = (VacuumEnvironmentState *)(getEnvironment()->readState());
    buffer << "=====VACUUM WORLD SIMULATION RESULTS=====\n";
    //TODO add info on when simulation was run
    buffer << "\tSimulation Cycles: " << s->getMaxCycles()<< "\n";
    buffer << "\tEnvironment Size: [" << state->getWidth() << "x" << state->getHeight() << " tiles]\n";
    buffer << "\tPerformance Measure: " << s->getEnvironment()->getPerformanceMeasure() << "\n";
    return buffer.str();
}

VacuumWorldSimulationResult::VacuumWorldSimulationResult(Simulator *sim) : SimulatorResult(sim) {

}
