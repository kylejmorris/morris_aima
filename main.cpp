#include <iostream>
#include <Agent.h>
#include <FrameVisualizer.h>
#include "TileEnvironment.h"
#include "Tile.h"

#include "TileGrid.h"
#include <QApplication>
#include <TileSimulator.h>
#include <TileFrameVisualizer.h>

using namespace std;

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);
    TileEnvironment *e = new TileEnvironment();
    Visualizer *v = new TileFrameVisualizer(4,4,"Real world!!!");
    e->loadEnvironment("config/testMap.map");

    TileSimulator *simulation = new TileSimulator(e,v,1000);
    simulation->start();
    return application.exec();
}
//TODO: note for future update: perhaps have simulator that only takes in a map name then does all the loading.
//not worth implementing yet as it's not needed; but I'll keep design things in mind for the future, only implementing
//when I'm sure it is beneficial/makes sense and isn't just me being compulsive.