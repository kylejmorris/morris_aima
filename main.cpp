#include <iostream>
#include <Agent.h>
#include <FrameVisualizer.h>
#include "TileEnvironment.h"
#include "Tile.h"

#include "TileGrid.h"
#include <QApplication>
#include "Simulator.h"
#include <TileFrameVisualizer.h>
#include <VacuumEnvironment.h>
#include <unistd.h>
#include <SimulatorFactory.h>
#include <sstream>

using namespace std;

void displayPrompt() {
    std::cout<< "\nCONFIG DETAILS:\n";
    std::cout<< "\t1: Vacuum at [0,0], NO DIRT.\n";
    std::cout<< "\t2: Vacuum at [0,0], Dirt at [0,0]\n";
    std::cout<< "\t3: Vacuum at [0,0], Dirt at [1,0]\n";
    std::cout<< "\t4: Vacuum at [0,0], Dirt at {[0,0], [1,0]}\n";
    std::cout<< "\t5: Vacuum at [1,0], NO DIRT.\n";
    std::cout<< "\t6: Vacuum at [1,0], Dirt at [0,0]\n";
    std::cout<< "\t7: Vacuum at [1,0], Dirt at [1,0]\n";
    std::cout<< "\t8: Vacuum at [1,0], Dirt at {[0,0, [1,0]}\n";
    std::cout << "\n----------------\n";
}
int main(int argc, char *argv[]) {
    int cycleSpeed = 500;
    int cycles = 10;
    int simulationId = 0; //between 0 and 8, which file to run
    QApplication application(argc, argv);
    std::ostringstream fileName;
    std::ostringstream frameName;
    std::cout << "Which simulation do you want to run for Vacuum World? (integer [0,8])";
    displayPrompt();
    std::cin >> simulationId;
    while(simulationId<1 || simulationId>8) {
        std::cout << "Invalid id, enter a proper one you goof.";
        displayPrompt();
        std::cin >> simulationId;
    }
    frameName << "Vacuum World Sim_" << simulationId;
    fileName<<"Projects/VacuumWorld/config/map"<< simulationId << ".map";
    Simulator *sim = SimulatorFactory::createSimulator("VacuumWorld",frameName.str(), fileName.str(), cycleSpeed);
    sim->start(cycles);
    return application.exec();
}

//TODO: note for future update: perhaps have simulator that only takes in a map name then does all the loading.
//not worth implementing yet as it's not needed; but I'll keep design things in mind for the future, only implementing
//when I'm sure it is beneficial/makes sense and isn't just me being compulsive.