/**
* CLASS: VacuumEnvironment
* DATE: 24/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: General vacuum environment support for the AIMA chapter 1 problems involving reflex agent,
* and a simple randomized agent.
*
* AGENTS (supported in environment)
*   VacuumAgent, RandomizedVacuumAgent
* ENTITIES (supported in environment):
*   Dirt: Objects that vacuums can clean up from tiles.
*   Wall: objects that vacuums can't pass through.
*
* Properties of Task Environment
*   SINGLE AGENT: Only 1 vacuum exists
*   DETERMINISTIC: The outcome of an agents actions are exactly as determined, no probabilistic events.
*   UNCERTAIN: Since environment is not fully observable, it's deemed uncertain.
*   EPISODIC: The agents experience is divided into atomic episodes. Each episode the agent receives a percept, and performs
*   an action. The next episode does not depend on the actions taken in previous episodes.
*   STATIC: The environment does not change while an Agent is thinking.
*   DISCRETE: Finite number of possible states in environment. Not continuous real number values.
*/

#ifndef MORRIS_AIMA_VACUUMENVIRONMENT_H
#define MORRIS_AIMA_VACUUMENVIRONMENT_H
#include <TileEnvironment.h>
#include "VacuumEnvironmentState.h"
#include <std_srvs/Empty.h>
#include "SimpleVacuumAction.h"
#include "VacuumWorldPerformanceMeasure.h"

class SimpleReflexVacuumAgent;
class RandomVacuumAction;
class VacuumEnvironment : public TileEnvironment {
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
     * When initialized, this will provide the service used to load the environment.
     * ie: will receive callbacks to load();
     */
    ros::ServiceServer loadService;

    /**
     * When initialized, this will provide the service used to reset the environment.
     * ie: will receive callbacks to reset();
     */
    ros::ServiceServer resetService;

    /**
     * Publishes state of VacuumWorld environment
     */
    //ros::Publisher statePublisher;

    /**
     * The state of the environment
     */
    VacuumEnvironmentState *state = NULL;

    /**
     * We know this environment contains only 1 vacuum, so we identify it here for easy access instead of searching grid.
     */
    VacuumAgent *vacuum = NULL;

    VacuumWorldPerformanceMeasure *performanceMeasure = NULL;
    /**
     * Where we found vacuum. Only valid if the Vacuum is found.
     */
    TileLocation *vacuumLocation = NULL;

public:
    virtual void publish();

    virtual void reset() override;

    virtual void initialize();

    virtual double getPerformanceMeasure();

    virtual EnvironmentState *readState() override;

public:
    VacuumEnvironment();

    /**
     * Percept is read in and fed into Vacuum agent, vacuum declares it's action and the state of environment is
     * updated.
     */
    virtual void act() override;

    /**
     * Nothing generated for this environment.
     */
    virtual void generate() override;

    virtual void loadEnvironment(string fileName) override;

    /**
     * We update performance measure and statistics.
     * Performance measure is based on the number of clean tiles
     */
    virtual void updateResults() override;

    virtual ~VacuumEnvironment();

    void simpleVacuumAct(SimpleVacuumAction *action);
    void randomVacuumAct(RandomVacuumAction *action);

    /**
     * Activate the environment so it may cycle. This just calls the parent Environment activate() routine and is used to provide the activate service to ros.
     */
    bool activate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);
    /**
     * Activate the environment so it won't cycle. This just calls the parent Environment deactivate() routine and is used to provide the activate service to ros.
     */
    bool deactivate_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * Load the environment. This will reset, and refresh the environment with new parameters specified
     */
    bool load_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    /**
     * Reset the environment. This will clear the cycle history and all statistics.
     */
    bool reset_callback(std_srvs::Empty::Request &req, std_srvs::Empty::Response &resp);

    virtual std::string outputToJson() override;
};

#endif //MORRIS_AIMA_VACUUMENVIRONMENT_H
