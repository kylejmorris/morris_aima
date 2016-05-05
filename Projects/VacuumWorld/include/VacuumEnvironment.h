/**
* CLASS: VacuumEnvironment
* DATE: 24/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: General vacuum environment support for the AIMA chapter 1 problems involving reflex agent,
* and such.
 * This map is a 2x1 grid (2 columns x 1 row) The map file must have sizes set accordingly for the agent to work!
 *
 * Properties of Task Environment
 *   PARTIALLY OBSERVABLE: The agent (vacuum) can only see it's current tile. Not the whole environment state.
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
#include "VacuumWorldPerformanceMeasure.h"

class VacuumAgent;
class VacuumEnvironment : public TileEnvironment {
private:
    /**
     * The state of the environment
     */
    VacuumEnvironmentState *state;
    /**
     * We know this environment contains only 1 vacuum, so we identify it here for easy access instead of searching grid.
     */
    VacuumAgent *vacuum;

    VacuumWorldPerformanceMeasure *performanceMeasure;
    /**
     * Where we found vacuum. Only valid if the Vacuum is found.
     */
    TileLocation *vacuumLocation;

public:
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
};

#endif //MORRIS_AIMA_VACUUMENVIRONMENT_H
