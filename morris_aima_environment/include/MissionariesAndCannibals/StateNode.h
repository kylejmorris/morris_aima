/**
* CLASS: StateNode
* DATE: 05/06/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A state node for Missionaries and Cannibals, as outlined in chapter 3
 * of AIMA. Contains properties of a node used for a simple search.
*/

#ifndef MORRIS_AIMA_STATENODE_H
#define MORRIS_AIMA_STATENODE_H

#include <MandCEnvironmentState.h>
#include "MandCAction.h"

class StateNode {
private:
    /**
     * How much it cost to get to this node.
     */
    int pathCost = 0;

    /**
     * The state corresponding to this node.
     */
    MandCEnvironmentState *state;

    StateNode *parent;

    /**
     * What action was taken to get to this node from the parent?
     */
    MandCAction *action;

public:
    bool operator==(const StateNode &node) const;

    int getPathCost() const {
        return pathCost;
    }

    void setPathCost(int pathCost) {
        StateNode::pathCost = pathCost;
    }

    MandCAction *getAction() const {
        return action;
    }

    void setAction(MandCAction *action) {
        StateNode::action = action;
    }

    StateNode *getParent() const {
        return parent;
    }

    void setParent(StateNode *parent) {
        StateNode::parent = parent;
    }

    MandCEnvironmentState *getState() const;

    void setState(MandCEnvironmentState *state) {
        StateNode::state = state;
    }

    StateNode(int pathCost, MandCEnvironmentState *state, StateNode *parent, MandCAction *action);

    int compareTo(StateNode *other);
};


#endif //MORRIS_AIMA_STATENODE_H
