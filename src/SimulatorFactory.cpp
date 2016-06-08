#include <VacuumEnvironment.h>
#include <TileFrameVisualizer.h>
#include <Simulator.h>
#include <MandCEnvironment.h>
#include <TerminalTextVisualizer.h>
#include "SimulatorFactory.h"

#define DEFAULT_CYCLE_TIME 1000
Simulator *SimulatorFactory::createSimulator(std::string type, std::string name, std::string mapFile, long cycleTime) {
    Simulator *created;
    Environment *environment;
    Visualizer *visualizer;

    if (type.compare("VacuumWorld") == 0) {
        environment = new VacuumEnvironment;
        environment->loadEnvironment(mapFile);
        VacuumEnvironmentState *state = dynamic_cast<VacuumEnvironmentState *>(environment->readState());
        visualizer = new TileFrameVisualizer(state->getWidth(), state->getHeight(), name);
        created = new Simulator(environment, visualizer, cycleTime);
    } else if (type.compare("RandomizedVacuumWorld") == 0) {
        environment = new VacuumEnvironment;
        environment->loadEnvironment(mapFile);
        VacuumEnvironmentState *state = dynamic_cast<VacuumEnvironmentState *>(environment->readState());
        visualizer = new TileFrameVisualizer(state->getWidth(), state->getHeight(), name);
        created = new Simulator(environment, visualizer, cycleTime);
    } else if(type.compare("MissionariesAndCannibals") == 0) {
        environment = new MandCEnvironment;
        environment->loadEnvironment("default"); //initialize it to default
        visualizer = new TerminalTextVisualizer;
        created = new Simulator(environment, visualizer, cycleTime);
    } else {
        created = NULL;
    }
    return created;
}

