#include <VacuumEnvironmentState.h>
#include "VacuumWorldPerformanceMeasure.h"

bool VacuumWorldPerformanceMeasure::update(EnvironmentState *state) {
    int sum = 0;
    VacuumEnvironmentState *vacuumState = dynamic_cast<VacuumEnvironmentState *>(state);
    for(int row=0; row<vacuumState->getWidth(); row++) {
        for(int col=0; col<vacuumState->getHeight(); col++) {
            if(vacuumState->isDirty(row, col)) {
                sum++;
            }
        }
    }
    this->measure+=sum;
}

double VacuumWorldPerformanceMeasure::getPerformanceMeasure() {
    return this->measure;
}
