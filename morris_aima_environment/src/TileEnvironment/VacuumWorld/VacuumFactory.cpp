#include "SimpleReflexVacuumAgent.h"
#include <DirtEntity.h>
#include <WallEntity.h>
#include "VacuumFactory.h"
#include "RandomReflexVacuumAgent.h"

Entity *VacuumFactory::createEntity(std::string name, Json::Value properties) {
    Entity *result = NULL;

    if(name.compare("RandomReflexVacuumAgent")==0) {
        result = new RandomReflexVacuumAgent;
    } else if(name.compare("SimpleReflexVacuumAgent")==0) {
        result = new SimpleReflexVacuumAgent;
    } else if(name.compare("Dirt")==0) {
        result = new DirtEntity;
    } else if(name.compare("Wall")==0) {
        result = new WallEntity;
    }

    return result;
}

Entity *VacuumFactory::createEntity(std::string name, XmlRpc::XmlRpcValue *properties) {
    Entity *result = NULL;

    if(name.compare("RandomReflexVacuumAgent")==0) {
        result = new RandomReflexVacuumAgent;
    } else if(name.compare("SimpleReflexVacuumAgent")==0) {
        result = new SimpleReflexVacuumAgent;
    } else if(name.compare("Dirt")==0) {
        result = new DirtEntity;
    } else if(name.compare("Wall")==0) {
        result = new WallEntity;
    }

    return result;
}


