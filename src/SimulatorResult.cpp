#include "SimulatorResult.h"
#include "Simulator.h"
#include <fstream>


SimulatorResult::SimulatorResult(Simulator *target) {
    this->simulation = target;
}

Simulator *SimulatorResult::getSimulator() {
    return this->simulation;
}

Environment *SimulatorResult::getEnvironment() {
    return getSimulator()->getEnvironment();
}

bool SimulatorResult::writeToFile() {
    //TODO finish this, can't while on a plane, need to have docs/etc.
/*    std::ofstream output;
    std::string fileName = "VacuumWorld_sim_output_TIMESTAMP.out";
    bool result = false;
    output.open(fileName);
*/
}

bool SimulatorResult::writeToFile(std::string outputFile) {
    std::ofstream output;
    bool result = false; //whether or not writing to file worked
    output.open(outputFile);
    if(output.is_open()) {
        output << getResults();
        output.flush();
        output.close();
        result = true;
    }
    return result;
}

Visualizer *SimulatorResult::getVisualizer() {
    return getSimulator()->getDisplay();
}
