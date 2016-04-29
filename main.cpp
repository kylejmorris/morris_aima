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

using namespace std;

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);
    TileEnvironment *e = new VacuumEnvironment();
    Visualizer *v = new TileFrameVisualizer(2,1,"Vacuum World");
    e->loadEnvironment("config/VacuumWorld_map1.map");

    Simulator *simulation = new Simulator(e, v, 1000);
    simulation->start(1000);
    return application.exec();
}
//TODO: note for future update: perhaps have simulator that only takes in a map name then does all the loading.
//not worth implementing yet as it's not needed; but I'll keep design things in mind for the future, only implementing
//when I'm sure it is beneficial/makes sense and isn't just me being compulsive.