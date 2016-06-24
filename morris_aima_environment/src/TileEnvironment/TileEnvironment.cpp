#include <iostream>
#include <fstream>
#include <EntityFactoryFactory.h>
#include <EnvironmentStateFactory.h>
#include <XmlRpcValue.h>
#include <ros/ros.h>
#include "TileEnvironment.h"
#include "json/json.h"
#include "Agent.h"

/**
 * Default width/height of environment, assuming no input file is given.
 */
static const int DEFAULT_WIDTH = 5;
static const int DEFAULT_HEIGHT = 5;

TileEnvironment::TileEnvironment() {
    this->state = new TileEnvironmentState(DEFAULT_WIDTH, DEFAULT_HEIGHT);
}

TileEnvironment::~TileEnvironment() {
    delete this->state;
}

void TileEnvironment::act() { }

void TileEnvironment::generate() { }

void TileEnvironment::updateResults() { }

std::string TileEnvironment::outputToJson() {
    Json::Value outputRoot;
    std::string result;
    //general environment info
    outputRoot["Environment"]["age"] = (int) getAge();
    outputRoot["Environment"]["size"]["x"] = state->getWidth();
    outputRoot["Environment"]["size"]["y"] = state->getHeight();

    //handling all entity info in the environment
    outputRoot["Environment"]["entities"] = Json::arrayValue;
    for (int row = 0; row < this->state->getHeight(); row++) {
        for (int col = 0; col < state->getWidth(); col++) {
            TileLocation location(col, row);
            std::vector<Entity *> onTile = state->getEntitiesAt(&location);
            for (auto current : onTile) {
                Json::Value currentEntity; //current entity on a tile we're making info for
                currentEntity["type"] = current->getType();
                currentEntity["id"] = current->getId();
                currentEntity["location"]["x"] = col;
                currentEntity["location"]["y"] = row;

                outputRoot["Environment"]["entities"].append(currentEntity);
            }
        }
    }
    result = outputRoot.toStyledString();
    return result;
}

bool TileEnvironment::load() {
    bool result = true; //true until proven otherwise
    TileEnvironmentState *state;
    int x, y;
    std::string environmentType;
    ros::param::get("/morris_aima_control/world_type", environmentType);
    XmlRpc::XmlRpcValue properties;
    getNodeHandle()->getParam("config", properties);
    if (!properties.hasMember("grid_width")) {
        result = false;
        ROS_INFO("ERROR: invalid parameter for grid width");
    } else {
        x = properties["grid_width"];
    }
    if (!properties.hasMember("grid_height")) {
        result = false;
        ROS_INFO("ERROR: invalid parameter for grid height");
    } else {
        y = properties["grid_height"];
    }

    if (!properties.hasMember("entities")) {
        result = false;
        ROS_INFO("ERROR: invalid parameter for entities. note: [] indicates no entities.");
    }
    //setting the factory for the given type of environment.
    this->entityFactory = EntityFactoryFactory::createEntityFactory(environmentType);

    //figure out which environment state is associated with this environment
    EnvironmentState *tempState = EnvironmentStateFactory::createEnvironmentState(environmentType, properties);
    state = static_cast<TileEnvironmentState *>(tempState);

    XmlRpc::XmlRpcValue entities; //list of entities to put in grid
    getNodeHandle()->getParam("config/entities", entities);
    if(!entities.valid()) {
        ROS_INFO("ERROR: entities configuration for environment is not valid or not set.");
    } else {
        for (int count = 0; count < entities.size(); count++) {
            std::string entityType = static_cast<std::string>(entities[count]["entity_type"]);
            XmlRpc::XmlRpcValue entityDetails = entities[count];
            Entity *current = this->entityFactory->createEntity(entityType, &entityDetails);
            TileLocation loc(entities[count]["location_x"], entities[count]["location_y"]);
            state->add(current, &loc);
        }
    }
    this->state = state;
    if(this->state==NULL) {
       result = false;
    }
    if(result) {
        loaded = true;
        ROS_INFO("Successfully loaded %d x %d environment!",x,y);
    } else {
        ROS_INFO("Failed to load the %d x %d environment!",x,y);
    }

    return result;
}

void TileEnvironment::reset() {
    Environment::reset();
    this->loaded = false;
}


bool TileEnvironment::isLoaded() {
    return this->loaded;
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
    if (!success) {
        std::cout << reader.getFormatedErrorMessages();
    } else {
        environmentType = root["Enviroment"].get("type", "VacuumEnvironment").asString(); //default is VacuumEnvironment

        //setting the factory for the given type of environment.
        this->entityFactory = EntityFactoryFactory::createEntityFactory(environmentType);

        tiles = root["Environment"]["tiles"];
        x = root["Environment"]["size"]["x"].asInt();
        y = root["Environment"]["size"]["y"].asInt();

        //figure out which environment state is associated with this environment
        EnvironmentState *tempState = EnvironmentStateFactory::createEnvironmentState(environmentType,
                                                                                      root["Environment"]);
        state = static_cast<TileEnvironmentState *>(tempState);
        for (auto entity : root["Environment"]["entities"]) {
            Entity *current = this->entityFactory->createEntity(entity["type"].asString(), entity["properties"]);
            TileLocation loc(entity["location"]["x"].asInt(), entity["location"]["y"].asInt());
            state->add(current, &loc);
        }
    }

    this->state = state;
}

EnvironmentState *TileEnvironment::readState() {
    return this->state;
}

TileLocation *TileEnvironment::getLocationOf(int id) {
    return this->state->getLocationOf(id);
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
    return this->state->getEntities();
}

std::vector<Entity *> TileEnvironment::getEntitiesAt(TileLocation *place) {
    return this->state->getEntitiesAt(place);
}
