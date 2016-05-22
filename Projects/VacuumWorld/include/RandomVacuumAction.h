/**
* CLASS: RandomVacuumAction
* DATE: 19/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: The actions a randomized reflex vacuum agent can perform.
*/

#ifndef MORRIS_AIMA_RANDOMVACUUMACTION_H
#define MORRIS_AIMA_RANDOMVACUUMACTION_H

#include <Action.h>

class RandomVacuumAction : public Action {
public:
    virtual std::string toString() override;

    enum ACTION {
        LEFT,
        RIGHT,
        UP,
        DOWN,
        SUCK
    };

    enum ACTION choice;

    RandomVacuumAction(enum RandomVacuumAction::ACTION choice);
};


#endif //MORRIS_AIMA_RANDOMVACUUMACTION_H
