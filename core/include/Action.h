/**
* CLASS: Action
* DATE: 25/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: An action that may be done within an Environment. The environment decides how to execute the action.
 * For example, a standard vacuum may have an action called "move" which will contain the location to move to.
 * Note that only agents can perform actions, as agents are by definition that in which can "act".
*/

#ifndef MORRIS_AIMA_ACTION_H
#define MORRIS_AIMA_ACTION_H
#include <string>

class Action {
public:
    /**
     * Give string representation of action.
     */
    virtual std::string toString() = 0;
};


#endif //MORRIS_AIMA_ACTION_H
