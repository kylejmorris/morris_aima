#include <VacuumEnvironmentState.h>
#include <json/json.h>
#include "VacuumEnvironmentState.h"
#include "EnvironmentStateFactory.h"

EnvironmentState *EnvironmentStateFactory::createEnvironmentState(std::string type, Json::Value properties) {
    EnvironmentState *selected = NULL;
    if (type.compare("vacuum_world") == 0) {
        int sizex = properties["size"]["x"].asInt();
        int sizey = properties["size"]["y"].asInt();
        selected = new VacuumEnvironmentState(sizex, sizey);
    } else {
        selected = NULL;
    }
    return selected;
}

EnvironmentState *EnvironmentStateFactory::createEnvironmentState(std::string type,
                                                                  XmlRpc::XmlRpcValue stateProperties) {
    EnvironmentState *selected = NULL;
    if (type.compare("vacuum_world") == 0) {
        int sizex = stateProperties["grid_width"];
        int sizey = stateProperties["grid_height"];
        selected = new VacuumEnvironmentState(sizex, sizey);
    } else {
        selected = NULL;
    }
    return selected;
}


