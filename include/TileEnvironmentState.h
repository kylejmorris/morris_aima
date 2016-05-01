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

    //Overridden
    virtual int compareTo(EnvironmentState *other) override;
    virtual bool isValid() override;

    //These methods are the exact same as in TileEnvironment class. Read about them there.
    virtual bool add(Entity *e, Location *location);
    virtual Entity * remove(int id);
    virtual bool exists(int id);
    virtual TileLocation *getLocationOf(int id);
    virtual int getWidth();
    virtual int getHeight();
    virtual std::vector<Entity *> getEntitiesAt(TileLocation *place);
    virtual std::vector<Entity *> getEntities();

    //routines specific to TileEnvironment State and it's subclasses
    /**
     * Move an entity with given id to a new location.
     * @param id: The ID of entity to move
     * @param newLocation: The new location to move entity to.
     * @return bool: True if entity was moved. False if it was unsuccessful (entity doesn't exist, or location is invalid)
     */
    virtual bool moveEntity(int id, TileLocation *newLocation);
};

#endif //MORRIS_AIMA_TILEENVIRONMENTSTATE_H
