#include "VacuumPercept.h"
#include "TileLocation.h"

VacuumPercept::VacuumPercept(TileLocation *loc, bool isDirty) {
    setIsDirty(isDirty);
    if(loc->getX()==0) {
        setTileName('A');
    } else {
        setTileName('B');
    }
}

std::string VacuumPercept::toString() {
    std::string result;
    if(tileName=='A') {
       result = "[A, ";
    } else {
        result = "[B, ";
    }

    if(isDirty()) {
        result = result + "Dirty]";
    } else {
        result = result + "Clean]";
    }

    return result;
}

char VacuumPercept::getTileName() {
    return tileName;
}

void VacuumPercept::setTileName(char tileName) {
    if(tileName=='A' || tileName=='B') {
        this->tileName = tileName;
    } else {
        this->tileName = 'A';
    }
}

bool VacuumPercept::isDirty()  {
    return dirty;
}

void VacuumPercept::setIsDirty(bool isDirty) {
   dirty = isDirty;
}
