/**
* CLASS: Agent
* DATE: 05/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: An Agent can act in an environment by receiving Percepts (state information)
* via sensors and acting through actuators. In most simulation cases, "actuators" are merely objects
* initialized within the Agent that will be intercepted by the invironment during a given cycle
* and interpreted there. The agent however is in control of receiving percepts and deciding on an action.
*/

#ifndef MORRIS_AIMA_AGENT_H
#define MORRIS_AIMA_AGENT_H

#include "Entity.h"

class Action;
class Percept;

class Agent : public Entity {
public:
    virtual int compareTo(Entity *other) override;
    virtual std::string toString() override;
    virtual std::string getType() override;

    /**
     * Given a new percept, have the agent decide on an action. An agents action is always based on some percept, or
     * percept sequence; but never on anything it has not perceived before.
     * NOTE: The action the agent decides may or may not succeed in the environment. In an unknown environment
     * the agent may not know how it's actions will be received, so it's up to the Environment to handle them.
     * @param given: The given percept, what the agent sees in the environment during a cycle
     * @return Action: The action the agent decides to do. NULL represents a NOP, meaning the agent is not acting.
     */
    virtual Action *think(Percept *given);
};


#endif //MORRIS_AIMA_AGENT_H
