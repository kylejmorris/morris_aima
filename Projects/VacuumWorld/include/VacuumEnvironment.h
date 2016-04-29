/**
* CLASS: VacuumEnvironment
* DATE: 24/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: General vacuum environment support for the AIMA chapter 1 problems involving reflex agent,
* and such.
 * This map is a 2x1 grid (2 columns x 1 row) The map file must have sizes set accordingly for the agent to work!
*/

#ifndef MORRIS_AIMA_VACUUMENVIRONMENT_H
#define MORRIS_AIMA_VACUUMENVIRONMENT_H
#include <TileEnvironment.h>

class VacuumEnvironment : public TileEnvironment {

public:
    VacuumEnvironment();

protected:
    virtual void act() override;

    virtual void generate() override;

    virtual void updateResults() override;

    virtual ~VacuumEnvironment();
};


#endif //MORRIS_AIMA_VACUUMENVIRONMENT_H
