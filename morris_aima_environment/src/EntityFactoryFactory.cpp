#include <VacuumEnvironment.h>
#include <VacuumFactory.h>
#include <TileEnvironment/VacuumWorld/VacuumFactory.h>
#include "EntityFactory.h"
#include "EntityFactoryFactory.h"

EntityFactory *EntityFactoryFactory::createEntityFactory(std::string type) {
    EntityFactory *result = NULL;

    if(type.compare("vacuum_world")==0) {
        result = new VacuumFactory;
    }

    return result;
}

