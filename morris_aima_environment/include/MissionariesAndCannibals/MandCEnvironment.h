/**
* CLASS: MandCEnvironment
* DATE: 05/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Runs the simulation by performing a search. This environment is kind of odd
* since there is actually no agent, it's the environment itself that's performing the search because that's what this
* problem is all about, finding the environment state; not about some agent finding it's way in the environment.
*/

#ifndef MORRIS_AIMA_MANDCENVIRONMENT_H
#define MORRIS_AIMA_MANDCENVIRONMENT_H

#include <Environment.h>
#include <queue>
#include <std_srvs/Empty.h>
#include "json/json.h"

class StateNode;
class MandCEnvironmentState;

class MandCEnvironment : public Environment {
private:
    /**
     * When initialized, this will provide the service used to activate the environment.
     * ie: will receive callbacks to activate()
     */
    ros::ServiceServer activateService;

    /**
     * When initialized, this will provide the service used to deactivate the environment.
     * ie: will receive callbacks to deactivate();
     */
    ros::ServiceServer deactivateService;

    /**
     * When initialized, this will provide the service used to reset the environment.
     * ie: will receive callbacks to reset();
     */
    ros::ServiceServer resetService;

    /**
     * When initialized, this will provide the service used to load the environment.
     * ie: will receive callbacks to reset();
     */
    ros::ServiceServer load_service;

    /**
     * The initial environment state we'll start searching from.
     */
    MandCEnvironmentState *initialState;
    /**
     * the goal state we are searching for.
     */
    MandCEnvironmentState *goalState;

    /**
     * The node containing the found goal state, used to backtrack and generate the path to goal
     */
    StateNode *goalNode = NULL;

    /**
     * The frontier used for our search. Implemented as a queue for BFS.
     * These are all the nodes that have been generated but not yet expanded.
     */
    std::vector<StateNode *> frontier;

    /**
     * Explored set, all the nodes that have been expanded.
     */
    std::vector<StateNode *> explored;

    //default publish that shows each cycle of environment.
    ros::Publisher environment_publisher;
public:
    /**
     * Setup the services and other ros components of this environment piece.
     */
    virtual void initialize() override;

    /**
     * Publish relevent ros info of the environment.
     */
    virtual void publish() override;

    /**
     * Activate the environment so it may cycle. This just calls the parent Environment activate() routine and is used to provide the activate service to ros.
     */
    bool activate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * load the environment so it may cycle. This just calls the parent Environment activate() routine and is used to provide the activate service to ros.
     */
    bool load_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * Activate the environment so it won't cycle. This just calls the parent Environment deactivate() routine and is used to provide the activate service to ros.
     */
    bool deactivate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * Reset the environment. This will clear the cycle history and all statistics.
     */
    bool reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    virtual EnvironmentState *readState() override;
    virtual std::string outputToJson() override;

    /**
     * Convert the state into a valid json statement.
     * Since we do this often to generate json output, it's easier to make this a separate method.
     */
    Json::Value outputStateToJson(MandCEnvironmentState *state);

    /**
     * We don't actually care about a fileName, we just initialize the default environment for this problem.
     * No maps needed.
     */
    virtual void loadEnvironment(string fileName) override;

    //Environment has no agents, so these do nothing but return NULL/false/dummyValues
    virtual bool add(Entity *e, Location *place) override;
    virtual Entity *remove(int id) override;
    virtual bool exists(int id) override;
    virtual std::vector<Entity *> getEntities() override;
    virtual double getPerformanceMeasure() override;
protected:
    /**
     * Run an iteration of Breadth first search on the nodes.
     */
    virtual void act() override;

    //nothing happening in these stages for this environment.
    virtual void generate() override;
    virtual void updateResults() override;

    //add state nodes to frontier or explored sets
    void addToFrontier(StateNode *target);

    void addToExplored(StateNode *target);

    /**
     * Check to see if a goal node has been set, meaning we found a goal.
     * True if found, false otherwise.
     */
    bool goalFound();

    /**
     * Perform all possible actions on the state node and return the resulting nodes generated.
     */
    std::vector<StateNode *> expandNode(StateNode *target);

    /**
     * Check if a given node is found in the frontier.
     */
    bool inFrontier(StateNode *node);

    /**
     * Check if a given node is found in the explored set
     */
    bool inExploredSet(StateNode *node);
};

#endif //MORRIS_AIMA_MANDCENVIRONMENT_H
