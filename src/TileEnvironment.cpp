#include <iostream>
#include <fstream>
#include "TileEnvironment.h"
#include "json/json.h"
#include "Agent.h"

/**
 * Default width/height of environment, assuming no input file is given.
 */
static const int DEFAULT_WIDTH = 5;
static const int DEFAULT_HEIGHT = 5;

TileEnvironment::TileEnvironment() {
    this->state = new TileEnvironmentState(DEFAULT_WIDTH,DEFAULT_HEIGHT);
}

TileEnvironment::~TileEnvironment() {
    delete this->state;
}

void TileEnvironment::loadEnvironment(string fileName) {
    TileEnvironmentState *state;
    int x, y;
    Json::Value currentEntities; //current entities on tile
    int entityCount; //how many entities of a given type were found on tile
    Json::Value root;
    Json::Value tiles; //pointers to all the tiles in map
    Json::Reader reader;
    std::string environmentType;
    std::ifstream inputFile(fileName);
    bool success = reader.parse(inputFile, root, false);
    if(!success) {
        std::cout << reader.getFormatedErrorMessages();
    } else {
        environmentType= root["Enviroment"].get("type", "TileEnvironment").asString();
        if(environmentType.compare("TileEnvironment")==0) {
            tiles = root["Environment"]["tiles"];
            x = root["Environment"]["size"]["x"].asInt();
            y = root["Environment"]["size"]["y"].asInt();
            state = new TileEnvironmentState(x, y);

            for (auto iterator : tiles) {
                x = iterator["x"].asInt();
                y = iterator["y"].asInt();
                currentEntities = iterator["contents"];
                entityCount = currentEntities.get("Agent", 0).asInt();
                for (int count = 0; count < entityCount; count++) {
                    state->add(new Agent, new TileLocation(x, y));
                }
            }
        }
    }
    this->state = state;
}

EnvironmentState *TileEnvironment::readState() {
    return this->state;
}

bool TileEnvironment::add(Entity *e, Location *place) {
    this->state->add(e, place);
}

Entity *TileEnvironment::remove(int id) {
    return this->state->remove(id);
}

bool TileEnvironment::exists(int id) {
    return this->state->exists(id);
}

std::vector<Entity *> TileEnvironment::getEntities() {
    std::vector<Entity *> result;
    std::vector<Entity *> onTile; //entities on a specific tile.
    TileLocation loc(0,0);

    for(int y=0; y<this->state->getHeight(); y++) {
       for(int x=0; x<this->state->getWidth(); x++) {
            loc = TileLocation(x,y); //current tile we're checking
            onTile = this->state->getEntitiesAt(&loc);
            result.insert(result.end(), onTile.begin(), onTile.end());
       }
    }

    return result;
}
