/**
* CLASS: EntityFactoryFactory
* DATE: 24/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Ladies and gentlemen, this is a Factor to create Factories. Specifically, this factory generates Entity factories.
 *
*/

#ifndef MORRIS_AIMA_ENTITYFACTORYFACTORY_H
#define MORRIS_AIMA_ENTITYFACTORYFACTORY_H
#include <string>

class EntityFactory;
class EntityFactoryFactory { //Thanks Mike. You could have stopped this; but it's too late now.
public:
    /**
     * Create an entity factory given the environment name.
     * @param type: Environment name
     * @return EntityFactory: The entity factory that will create all required entities for a given environment.
     */
    static EntityFactory *createEntityFactory(std::string type);
};


#endif //MORRIS_AIMA_ENTITYFACTORYFACTORY_H
