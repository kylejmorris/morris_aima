/**
 * Class: Environment
 * Author: Kyle Morris
 *
 * Abstract environment that acts as interface for all kinds of environments.
 * In general an environment will contains entities and a "moment" which
 * is essentially one cycle of the environment.
 */
#ifndef MORRIS_AIMA_ENVIRONMENT_H
#define MORRIS_AIMA_ENVIRONMENT_H
#include <string>

using namespace std;

class Environment {
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
    virtual EnvironmentState *readState();

    /**
     * Add a new entity to the environment at a desired location.
     * Depending on the environment, the Entity and Location may vary. For example
     * in a 2D grid a Location may consist of (x,y) points, where as in a 3D environment
     * you would have (x,y,z).
     * @param Entity *e: The entity you wish to add to the environment.
     * @param Location *place: The location to add this entity to.
     */
    virtual bool add(Entity *e, Location *place);
};

#endif //MORRIS_AIMA_ENVIRONMENT_H
