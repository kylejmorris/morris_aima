/**
* CLASS: MandCEnvironmentState
* DATE: 04/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A State for the missionaries and cannibals environment.
* There are 3 missionaries and 3 cannibals.
* There must never be less missionaries than cannibals on a given side of the river,
* or the missionaries will be eaten. The goal is to have everyone on the right side
* of the river.
*/

#ifndef MORRIS_AIMA_MANDCENVIRONMENTSTATE_H
#define MORRIS_AIMA_MANDCENVIRONMENTSTATE_H

#include <EnvironmentState.h>
class MandCEnvironmentState : public EnvironmentState {
private:
    int cannibalsLeft;
    int cannibalsRight;
    int missionariesLeft;
    int missionariesRight;

//false if the boat is on left side, true if we crossed over.
    bool riverCrossed;
public:
    int getCannibalsLeft() const {
        return cannibalsLeft;
    }

    void setCannibalsLeft(int cannibalsLeft) {
        MandCEnvironmentState::cannibalsLeft = cannibalsLeft;
    }

    int getCannibalsRight() const {
        return cannibalsRight;
    }

    void setCannibalsRight(int cannibalsRight) {
        MandCEnvironmentState::cannibalsRight = cannibalsRight;
    }

    int getMissionariesLeft() const {
        return missionariesLeft;
    }

    void setMissionariesLeft(int missionariesLeft) {
        MandCEnvironmentState::missionariesLeft = missionariesLeft;
    }

    int getMissionariesRight() const {
        return missionariesRight;
    }

    void setMissionariesRight(int missionariesRight) {
        MandCEnvironmentState::missionariesRight = missionariesRight;
    }

    bool isRiverCrossed() const {
        return riverCrossed;
    }

    void setRiverCrossed(bool riverCrossed) {
        MandCEnvironmentState::riverCrossed = riverCrossed;
    }

    /**
     * Set initial state with all 3 missionaries and cannibals on left side,
     * there isn't really a need to provide configuration for different starting states.
     */
    MandCEnvironmentState();

    //copy constructor
    MandCEnvironmentState(MandCEnvironmentState &copy);

    virtual bool isValid();
    /**
     * Check if states are equal.
     * @return: 0 if equal, nonzero if otherwise
     */
    virtual int compareTo(EnvironmentState *other) override;

    /**
     * Send missionaries and cannibals over the river.
     * A maximum of 2 people can fit on the ship at once.
     * @returns MandCEnvironmentState: Updated state with this action complete. If the resulting state is invalid, then return NULL.
     */
    MandCEnvironmentState *ship(int numCannibals, int numMissionaries);
};
#endif //MORRIS_AIMA_MANDCENVIRONMENTSTATE_H
