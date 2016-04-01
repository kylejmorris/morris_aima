/**
 * Class: Environment
 * Author: Kyle Morris
 *
 * Abstract environment that acts as interface for all kinds of environments.
 * In general an environment will contains entities and a "moment" which
 * is essentially one cycle of the environment. The environment will manage itself by manipulating
 * an EnvironmentState object associated with it.
 * The Environment is also responsbile for acting as a controller between Agents and
 * it's state. When agents declare actions for example, the Environment will determine how they
 * affect it's state.
 */
#ifndef MORRIS_AIMA_ENVIRONMENT_H
#define MORRIS_AIMA_ENVIRONMENT_H
#include <string>
#include "EnvironmentState.h"
#include "Entity.h"
#include "Location.h"
#include <vector>

using namespace std;

class Environment {
private:
    /**
     * How many cycles the environment has been through (ie calls to moment())
     */
    long age = 0;
public:
    /**
     * Initialize the environment from a map file, setting initial environment state.
     * @param string fileName: Name of the environment to load.
     */
    virtual void loadEnvironment(string fileName) = 0;

    /**
     * Return the state of the environment.
     * @returns: EnvironmentState - this is a current snapshot of the environment.
     */
    virtual EnvironmentState *readState() = 0;

    /**
     * Add a new entity to the environment at a desired location.
     * Depending on the environment, the Entity and Location may vary. For example
     * in a 2D grid a Location may consist of (x,y) points, where as in a 3D environment
     * you would have (x,y,z).
     * @param Entity *e: The entity you wish to add to the environment.
     * @param Location *place: The location to add this entity to.
     */
    virtual bool add(Entity *e, Location *place) = 0;

    /**
     * Remove an entity from the environment with specified id.
     * @param id: The id of entity to remove from environment.
     * @return Entity: give pointer to entity if entity was removed, null otherwise.
     */
    virtual Entity * remove(int id) = 0;

    /**
     * Determine if Entity with given id exists in the environment.
     * @param id: The id of entity to search for in environment.
     * @return bool: true if it does exist, false otherwise
     */
    virtual bool exists(int id) = 0;

    /**
     * Return a list of all entities in the environment.
     * @return: vector containing pointer to all entities in environment.
     */
    virtual std::vector<Entity *> getEntities() = 0;

    /**
     * Simple getter, return age of environment. All subclasses will simply call this
     * and don't need to implement their own.
     */
    long getAge();
};

#endif //MORRIS_AIMA_ENVIRONMENT_H
