#include <VacuumEnvironmentState.h>
#include <json/json.h>
#include "EnvironmentStateFactory.h"

EnvironmentState *EnvironmentStateFactory::createEnvironmentState(std::string type, Json::Value properties) {
    EnvironmentState *selected = NULL;
    if(type.compare("VacuumEnvironment")==0) {
        int sizex = properties["size"]["x"].asInt();
        int sizey = properties["size"]["y"].asInt();

        selected = new VacuumEnvironmentState(sizex, sizey);
    } else {
        selected = NULL;
    }
   return selected;
}
