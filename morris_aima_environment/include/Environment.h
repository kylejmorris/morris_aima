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
#include <ros/service_server.h>
#include "Location.h"
#include "PerformanceMeasure.h"
#include <vector>
#include <ros/node_handle.h>

using namespace std;

class Environment {
private:
    /**
     * How many cycles the environment has been through (ie calls to cycle())
     */
    long age = 0;

    /**
     * Whether or not the environment is currently active or not. If it's true (active) then
     * we will allow cycling. Otherwise assume the environment is paused or under some sort of analysis.
     */
    bool active = false;

private:
/**
     * The node handle for the node running this environment.
     */
    ros::NodeHandle *nodeHandle;
public:
    /**
     * Set up the environment services/etc to advertise
     * @param nh: The nodehandle for environment node we're using.
     */
    virtual void initialize() = 0;

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

    ros::NodeHandle *getNodeHandle() const;

    void setNodeHandle(ros::NodeHandle *nodeHandle);

    /**
     * Output all relevant state information of the environment into a string. By default this is json formatted
     * Note that the output of an environment may have more details than an initial loading map. For example, the age of Environment
     * will be part of it's state, although we don't load this by default as it's always initialized to 0.
     * @return string: the json string output of the environment.
     */
    virtual std::string outputToJson() = 0;

    /**
     * Return performance measure associated with this environment.
     * @return double: the performance measure in float form for this environment.
     */
    virtual double getPerformanceMeasure() = 0;
    /**
     * A cycle in the Environment will involve updating states, performance measures, and so forth.
     * This is a baseclass routine since it will call upon overridden methods from child Environments.
     */
    void cycle();
    //TODO implement a getStatistics method to return some EnvironmentStatistics object.

     /**
     * Determine if environment is active(able to be cycled) or not(paused).
     * @return bool: true if active, false otherwise
     */
    bool isActive();

    /**
     * Flag the environment so it may be cycled.
     */
    bool activate();

    /**
     * Flag to prevent cycling of the environment.
     */
    bool deactivate();

    /**
     * Have environment publish it's state on desired topic. This is using ros.
     */
    virtual void publish() = 0;

    /**
     * Stop the environment from cycling and put it back in it's original state. This may be some default state,
     * or a configuration specified before.
     */
     virtual void reset();
protected:
    /**
     * Run through all agents in environment and let them act.
     */
    virtual void act() = 0;

    /**
     * Phase involving generation of non agents, ie other factors in the environment that may change every cycle
     * randomly, or in other words they alter irregardless of the agents actions. For example random dirt generation
     * could occur here.
     */
    virtual void generate() = 0;

    /**
     * Performance measures and any other statistics of the environment are updated here.
     */
    virtual void updateResults() = 0;

};

#endif //MORRIS_AIMA_ENVIRONMENT_H
