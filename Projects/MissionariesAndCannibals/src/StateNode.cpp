#include "StateNode.h"

StateNode::StateNode(int pathCost, MandCEnvironmentState *state, StateNode *parent, MandCAction *action) {
    setPathCost(pathCost);
    setState(state);
    setParent(parent);
    setAction(action);
}
