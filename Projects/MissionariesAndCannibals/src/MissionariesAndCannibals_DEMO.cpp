#include <iostream>
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
    std::cout << "NO PROMPT TO DISPLAY: RUNNING SIMULATION.\n";
}

int main(int argc, char *argv[]) {
    int cycleSpeed = 800;
    int cycles = 10;
    int simulationId = 0; //between 0 and 8, which file to run
    QApplication application(argc, argv);
    displayPrompt();
    Simulator *sim = SimulatorFactory::createSimulator("MissionariesAndCannibals","Basic Missionaries and Cannibals","nomap", cycleSpeed);
    sim->start(cycles);
    return application.exec();
}
