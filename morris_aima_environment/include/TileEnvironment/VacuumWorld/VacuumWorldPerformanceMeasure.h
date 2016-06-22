/**
* CLASS: VacuumWorldPerformanceMeasure
* DATE: 01/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Performance measure for a vacuum world is a sum of all clean tiles in a given state.
*/

#ifndef MORRIS_AIMA_VACUUMWORLDPERFORMANCEMEASURE_H
#define MORRIS_AIMA_VACUUMWORLDPERFORMANCEMEASURE_H
#include <PerformanceMeasure.h>

class VacuumWorldPerformanceMeasure : public PerformanceMeasure {
private:
    double performanceMeasureValue = 0;
public:
    virtual double getPerformanceMeasure() override;

    virtual bool update(EnvironmentState *state);
};

#endif //MORRIS_AIMA_VACUUMWORLDPERFORMANCEMEASURE_H
