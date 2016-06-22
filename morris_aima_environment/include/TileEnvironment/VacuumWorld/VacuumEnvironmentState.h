/**
* CLASS: VacuumEnvironmentState
* DATE: 30/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: State of a vacuum environment.
*/

#ifndef MORRIS_AIMA_VACUUMENVIRONMENTSTATE_H
#define MORRIS_AIMA_VACUUMENVIRONMENTSTATE_H

#include <TileEnvironment.h>
#include "TileEnvironmentState.h"
#include "VacuumAgent.h"

class SimpleReflexVacuumAgent;

class VacuumEnvironmentState : public TileEnvironmentState {
public:
/**
 * Locate the vacuum within environment state. Important to do this so we have agent to operate on, before simulation starts for real.
 * @return SimpleReflexVacuumAgent: the vacuum we found in environment state
 */
    VacuumAgent *findVacuum();

    VacuumEnvironmentState(int width, int height);

    /**
     * Move the Environment's vacuum to a given location on the grid.
     * @param x/y: The target location to put vacuum on.
     * @return bool: true if move was successful, false otherwise
     */
    virtual bool moveVacuum(int x, int y);

    /**
     * Determine if a given tile has a wall on it or not.
     * @return bool: true if there is a wall, false otherwise
     */
    bool hasWall(int x, int y);
    /**
    * Determine if a given tile is dirty or not.
    * @return bool: True if tile contains turn, false otherwise.
    */
    bool isDirty(int x, int y);

    /**
     * Remove dirt from a given tile.
     * @return bool: True if we cleaned successfully, as in dirt was removed. False otherwise
     */
    bool cleanTile();
};

#endif //MORRIS_AIMA_VACUUMENVIRONMENTSTATE_H
