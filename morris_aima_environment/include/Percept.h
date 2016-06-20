/**
* CLASS: Percept
* DATE: 25/04/16
* AUTHOR: ${AUTHOR}
* DESCRIPTION: The term percept refers to an agents perceptual inputs at any given instance. Thus, a percept sequence
 * would be the complete history of evertyhing the agent has ever perceived. An agents action may only depend on any given percept,
 * or percept sequence, but on nothing that has not been perceived yet.
*/
#ifndef MORRIS_AIMA_PERCEPT_H
#define MORRIS_AIMA_PERCEPT_H
#include <string>

class Percept {
public:
    virtual std::string toString() = 0;
};


#endif //MORRIS_AIMA_PERCEPT_H
