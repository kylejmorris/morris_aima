/**
* CLASS: VacuumAction
* DATE: 29/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: The action made by a vacuum in a Vacuum world. Very simple, we just identify some names of the actions
 * and let it call them.
*/

#ifndef MORRIS_AIMA_VACUUMACTION_H
#define MORRIS_AIMA_VACUUMACTION_H
#include <Action.h>

class VacuumAction : public Action {
public:
    virtual std::string toString() override;

    enum ACTION {
        LEFT,
        RIGHT,
        SUCK
    };

    enum ACTION choice;

    VacuumAction(enum VacuumAction::ACTION choice);
};

#endif //MORRIS_AIMA_VACUUMACTION_H
