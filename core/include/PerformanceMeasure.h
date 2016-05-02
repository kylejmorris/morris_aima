/**
* CLASS: PerformanceMeasure
* DATE: 01/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A performance measure is a way of determining how well an Agent (or Agents) have performed
 * in an environment, as an Objective calculation.
*/

#ifndef MORRIS_AIMA_PERFORMANCEMEASURE_H
#define MORRIS_AIMA_PERFORMANCEMEASURE_H

class EnvironmentState;
class PerformanceMeasure {
public:
    /**
     * Update internal performance measure value given the current state of an environment.
     */
    virtual bool update(EnvironmentState *state) = 0;

    virtual double getPerformanceMeasure() = 0;
};


#endif //MORRIS_AIMA_PERFORMANCEMEASURE_H
