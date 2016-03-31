#include <iostream>
#include <Agent.h>
#include <FrameVisualizer.h>
#include "TileEnvironment.h"
#include "Tile.h"

#include "TileGrid.h"
#include <QApplication>

using namespace std;

int main(int argc, char *argv[]) {
    TileEnvironment *e = new TileEnvironment;
    e->loadEnvironment("config/testMap.map");
    e->add(new Agent(), new TileLocation(2, 3));
    std::vector<Entity *> entities = e->getEntities();

    QApplication application(argc, argv);
    FrameVisualizer *v = new FrameVisualizer(500,500, "Test visualizer");
    std::cout << "waiting for input...";
    std::string text;
    std::cin >> text;
    v->render();
    return application.exec();
}