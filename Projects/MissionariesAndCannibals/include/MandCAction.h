/**
* CLASS: MandCAction
* DATE: 05/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Represents a shipment action that can be taken in the environment for missionaries and cannibals.
* This is contained with nodes to show what exactly is done to transition between environment states.
*/

#ifndef MORRIS_AIMA_MANDCACTION_H
#define MORRIS_AIMA_MANDCACTION_H

#include <Action.h>
class MandCAction : public Action {
private:
    int numMissionaries;
    int numCannibals;

public:
    MandCAction(int numMissionaries, int numCannibals);

    virtual std::string toString();

    int getNumMissionaries() const {
        return numMissionaries;
    }

    void setNumMissionaries(int numMissionaries) {
        MandCAction::numMissionaries = numMissionaries;
    }

    int getNumCannibals() const {
        return numCannibals;
    }

    void setNumCannibals(int numCannibals) {
        MandCAction::numCannibals = numCannibals;
    }
};

#endif //MORRIS_AIMA_MANDCACTION_H
