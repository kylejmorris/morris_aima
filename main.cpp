#include <iostream>
#include <Agent.h>
#include <FrameVisualizer.h>
#include "TileEnvironment.h"
#include "Tile.h"

#include "TileGrid.h"
#include <QApplication>
#include <TileSimulator.h>

using namespace std;

int main(int argc, char *argv[]) {
    QApplication application(argc, argv);
    TileEnvironment *e = new TileEnvironment;
    e->loadEnvironment("config/testMap.map");
    e->add(new Agent(), new TileLocation(2, 3));
    std::vector<Entity *> entities = e->getEntities();

    //TileSimulator *simulation = new TileSimulator();
    //simulation->start();
    //return application.exec();
    return 0;
}