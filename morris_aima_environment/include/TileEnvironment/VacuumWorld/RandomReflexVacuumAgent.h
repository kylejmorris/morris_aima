/**
* CLASS: RandomReflexAgent
* DATE: 12/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A randomized agent just chooses at random which action to make. It doesn't regard what is allowed or should be done.
*/
#ifndef MORRIS_AIMA_RANDOMREFLEXAGENT_H
#define MORRIS_AIMA_RANDOMREFLEXAGENT_H
#include <VacuumAgent.h>

class RandomReflexVacuumAgent : public VacuumAgent {
public:
    virtual int compareTo(Entity* other) override;
    virtual std::string toString() override;
    virtual std::string getType() override;
    virtual Action* think(Percept* given) override;
};

#endif //MORRIS_AIMA_RANDOMREFLEXAGENT_H
