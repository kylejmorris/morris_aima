#include "SimulatorResult.h"
#include "Simulator.h"


SimulatorResult::SimulatorResult(Simulator *target) {
    this->simulation = target;
}

Simulator *SimulatorResult::getSimulator() {
    return this->simulation;
}

Environment *SimulatorResult::getEnvironment() {
    return getSimulator()->getEnvironment();
}

Visualizer *SimulatorResult::getVisualizer() {
    return getSimulator()->getDisplay();
}
