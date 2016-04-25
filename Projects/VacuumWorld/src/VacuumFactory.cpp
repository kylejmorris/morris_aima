#include <VacuumAgent.h>
#include "VacuumFactory.h"

Entity *VacuumFactory::createEntity(std::string name, Json::Value properties) {
    Entity *result = NULL;

    if(name.compare("VacuumAgent")==0) {
        result = new VacuumAgent;
    } else if(name.compare("Agent")==0) {
        result = new Agent;
    }

    return result;
}
