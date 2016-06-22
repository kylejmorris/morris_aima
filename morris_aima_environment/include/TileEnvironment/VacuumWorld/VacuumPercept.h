/**
* CLASS: VacuumPercept
* DATE: 29/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: All environment info perceived by a Vacuum agent at a given moment in time within a VacuumWorld.
*/
#ifndef MORRIS_AIMA_VACUUMPERCEPT_H
#define MORRIS_AIMA_VACUUMPERCEPT_H
#include "Percept.h"

class TileLocation;
class VacuumPercept : public Percept {
private:
    /**
     * A or B, the name of tile being perceived.
     */
    char tileName;

    /**
     * Whether or not the tile has dirt on it.
     */
    bool dirty;
public:
    VacuumPercept(TileLocation *loc, bool isDirty);

    char getTileName();

    void setTileName(char tileName);

    bool isDirty();

    void setIsDirty(bool isDirty);

    virtual std::string toString() override;
};


#endif //MORRIS_AIMA_VACUUMPERCEPT_H
