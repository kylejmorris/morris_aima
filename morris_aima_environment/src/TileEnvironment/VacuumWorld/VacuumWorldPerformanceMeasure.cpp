#include <VacuumEnvironmentState.h>
#include "VacuumWorldPerformanceMeasure.h"

bool VacuumWorldPerformanceMeasure::update(EnvironmentState *state) {
    int sum = 0;
    VacuumEnvironmentState *vacuumState = dynamic_cast<VacuumEnvironmentState *>(state);
    for(int row=0; row<vacuumState->getHeight(); row++) {
        for(int col=0; col<vacuumState->getWidth(); col++) {
            if(!vacuumState->isDirty(col, row)) {
                sum++;
            }
        }
    }
    this->performanceMeasureValue = getPerformanceMeasure() + sum;
    return true;
}

double VacuumWorldPerformanceMeasure::getPerformanceMeasure() {
    return this->performanceMeasureValue;
}
