/**
* CLASS: TileEnvironmentState
* DATE: 03/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A TileEnvironment will have a grid of tiles containing entities.
*/

#ifndef MORRIS_AIMA_TILEENVIRONMENTSTATE_H
#define MORRIS_AIMA_TILEENVIRONMENTSTATE_H
#include "EnvironmentState.h"
#include "TileGrid.h"

class TileEnvironmentState : public EnvironmentState {
private:
    /**
     * 2D grid of all the tiles.
     */
    TileGrid *tiles;

    const int WIDTH;
    const int HEIGHT;
public:
    TileEnvironmentState(int width, int height);
    virtual ~TileEnvironmentState();
    virtual bool isValid() override;
    virtual int compareTo(EnvironmentState *other) override;
    //These methods are the exact same as in TileEnvironment class. Read about them there.
    virtual bool add(Entity *e, Location *location);
    virtual Entity * remove(int id);
    virtual bool exists(int id);
};

#endif //MORRIS_AIMA_TILEENVIRONMENTSTATE_H
