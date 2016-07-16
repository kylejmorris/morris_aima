#include <VacuumEnvironment.h>
#include <MandCEnvironment.h>
#include "EnvironmentFactory.h"

Environment *EnvironmentFactory::createEnvironment(std::string name, XmlRpc::XmlRpcValue &properties) {
    Environment *result = NULL;
    if(name.compare("vacuum_world")==0) {
        result = new VacuumEnvironment;
    } else if(name.compare("missionaries_and_cannibals")==0) {
        result = new MandCEnvironment;

    }

    return result;
}

