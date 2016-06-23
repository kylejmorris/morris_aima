#include <VacuumEnvironment.h>
#include "EnvironmentFactory.h"

Environment *EnvironmentFactory::createEnvironment(std::string name, XmlRpc::XmlRpcValue &properties) {
    Environment *result = NULL;
    if(name.compare("vacuum_world")==0) {
        result = new VacuumEnvironment;
    }
    return result;
}

