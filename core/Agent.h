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

class Agent : public Entity {
public:
    virtual int compareTo(Entity *other) override;
    virtual std::string toString() override;
};


#endif //MORRIS_AIMA_AGENT_H
