#include "StateNode.h"
#include "MandCEnvironmentState.h"

StateNode::StateNode(int pathCost, MandCEnvironmentState *state, StateNode *parent, MandCAction *action) {
    setPathCost(pathCost);
    setState(state);
    setParent(parent);
    setAction(action);
}

int StateNode::compareTo(StateNode *other) {
    int equal = 0; //0 means yes
    equal += getState()->compareTo(other->getState());
    /*
    //ensure it's the same branch too
    if(getParent()!=NULL && getParent()->compareTo(other->getParent())!=0) {
        equal++;
    } else {
        if(other->getParent()!=NULL) {
            equal++;
        }
    }**/
    return equal;
}

bool StateNode::operator==(const StateNode &node) const {
    return true;
}


MandCEnvironmentState *StateNode::getState() const {
    return this->state;
}

