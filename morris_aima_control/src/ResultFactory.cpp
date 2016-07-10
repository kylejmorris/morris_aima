#include <VacuumWorldResult.h>
#include "ResultFactory.h"

Result* ResultFactory::createResult(std::string type, ros::NodeHandle *handle, XmlRpc::XmlRpcValue properties) {
    Result *created = NULL;

    if(type.compare("vacuum_world")==0) {
        created = new VacuumWorldResult(handle);
    }

    return created;
}