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
#include "json/json.h"

class StateNode;
class MandCEnvironmentState;

class MandCEnvironment : public Environment {
private:
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
    StateNode *goalNode;

    /**
     * The frontier used for our search. Implemented as a queue for BFS.
     * These are all the nodes that have been generated but not yet expanded.
     */
    std::vector<StateNode *> frontier;

    /**
     * Explored set, all the nodes that have been expanded.
     */
    std::vector<StateNode *> explored;

public:
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
};

#endif //MORRIS_AIMA_MANDCENVIRONMENT_H
