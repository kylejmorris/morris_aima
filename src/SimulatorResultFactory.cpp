#include <VacuumWorldSimulationResult.h>
#include "SimulatorResultFactory.h"

SimulatorResult *SimulatorResultFactory::createSimulatorResult(std::string type, Simulator *sim) {
    SimulatorResult *result = NULL;
    if(type.compare("VacuumWorld")==0) {
        result = new VacuumWorldSimulationResult(sim);
    }
    return result;
}
