#include <VacuumEnvironment.h>
#include <TileFrameVisualizer.h>
#include <Simulator.h>
#include "SimulatorFactory.h"

#define DEFAULT_CYCLE_TIME 1000
Simulator *SimulatorFactory::createSimulator(std::string name) {
    Simulator *created;

    if(name.compare("VacuumWorld")==0) {
        created = new Simulator(new VacuumEnvironment, new TileFrameVisualizer(2,1, "Vacuum World!"), DEFAULT_CYCLE_TIME);
    } else {
        created = NULL;
    }
    return created;
}

