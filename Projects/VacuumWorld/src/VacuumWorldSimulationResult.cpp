#include <sstream>
#include <VacuumEnvironment.h>
#include <fstream>
#include "VacuumWorldSimulationResult.h"
#include "Simulator.h"
#include <ctime>

std::string VacuumWorldSimulationResult::getResults() {
    std::ostringstream buffer;
    Simulator *s = getSimulator();
    VacuumEnvironmentState *state = (VacuumEnvironmentState *)(getEnvironment()->readState());
    time_t currTime = time(0);   // get time now
    struct tm * now = localtime( & currTime );
    buffer << "=====VACUUM WORLD SIMULATION RESULTS=====\n";
    buffer << "\tRan ON:";
    buffer << (now->tm_year + 1900) << '-' << (now->tm_mon + 1) << '-' << now->tm_mday;
    buffer << " at " << (now->tm_hour) << ":" << (now->tm_min) << ":" << (now->tm_sec) << "\n";
    buffer << "\tSimulation Cycles: " << s->getMaxCycles()<< "\n";
    buffer << "\tEnvironment Size: [" << state->getWidth() << "x" << state->getHeight() << " tiles]\n";
    buffer << "\tPerformance Measure: " << s->getEnvironment()->getPerformanceMeasure() << "\n";
    buffer << "\n";
    return buffer.str();
}

bool VacuumWorldSimulationResult::writeToFile(std::string outputFile) {
    std::ofstream output;
    bool result = false; //whether or not writing to file worked
    output.open(outputFile, std::ios_base::app);
    if(output.is_open()) {
        output << getResults();
        output.flush();
        output.close();
        result = true;
    }
    return result;
}

VacuumWorldSimulationResult::VacuumWorldSimulationResult(Simulator *sim) : SimulatorResult(sim) {
}
