#include <sstream>
#include "MandCAction.h"

MandCAction::MandCAction(int numMissionaries, int numCannibals) {
    setNumMissionaries(numMissionaries);
    setNumCannibals(numCannibals);
}

std::string MandCAction::toString() {
    std::ostringstream out;
    out << "SHIP: " << getNumMissionaries() << " Missionaries and " << getNumCannibals() << " cannibals.";
    return out.str();
}

