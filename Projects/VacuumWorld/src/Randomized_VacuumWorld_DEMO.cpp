#include <QApplication>
#include <iostream>
#include <sstream>
#include <Simulator.h>
#include <SimulatorFactory.h>

void displayPrompt() {
    std::cout<< "\nCONFIG DETAILS:\n";
    std::cout<< "\t1: Small 2x2 World with randomized agent\n";
    std::cout<< "\t2: Medium 4x4 World with randomized agent\n";
    std::cout<< "\t3: Big 10x10 World with randomized agent\n";
    std::cout << "\n----------------\n";
}

int main(int argc, char *argv[]) {
    int cycleSpeed = 1200;
    int cycles = 200;
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

    switch(simulationId) {
        case 1: frameName << "2x2 Randomized Vacuum World.";
            fileName << "Projects/VacuumWorld/config/wall_world_small_map1.map"; break;
        case 2: frameName << "4x4 Randomized Vacuum World. {medium}";
            fileName << "Projects/VacuumWorld/config/wall_world_medium_map1.map"; break;
        case 3: frameName << "10x10 Randomized Vacuum World. {large}";
            fileName << "Projects/VacuumWorld/config/wall_world_large_map1.map"; break;
    }

    Simulator *sim = SimulatorFactory::createSimulator("RandomizedVacuumWorld",frameName.str(), fileName.str(), cycleSpeed);
    sim->start(cycles);

    return application.exec();
}
