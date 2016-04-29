#include <VacuumAgent.h>
#include <DirtEntity.h>
#include "VacuumFactory.h"

Entity *VacuumFactory::createEntity(std::string name, Json::Value properties) {
    Entity *result = NULL;

    if(name.compare("VacuumAgent")==0) {
        result = new VacuumAgent;
    } else if(name.compare("Dirt")==0) {
        result = new DirtEntity;
    }

    return result;
}
