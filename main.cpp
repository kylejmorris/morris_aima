#include <iostream>
#include <Agent.h>
#include "TileEnvironment.h"
#include "Tile.h"
#include "TileGrid.h"

using namespace std;

int main() {
    TileEnvironment *e = new TileEnvironment;
    e->loadEnvironment("config/testMap.map");
    e->add(new Agent(), new TileLocation(2, 3));
    std::vector<Entity *> entities = e->getEntities();
    std::cout << entities.size();

    return 0;
}