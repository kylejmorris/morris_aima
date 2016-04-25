#include <VacuumEnvironment.h>
#include <VacuumFactory.h>
#include "EntityFactory.h"
#include "EntityFactoryFactory.h"

EntityFactory *EntityFactoryFactory::createEntityFactory(std::string type) {
    EntityFactory *result = NULL;

    if(type.compare("VacuumEnvironment")==0) {
        result = new VacuumFactory;
    }

    return result;
}

