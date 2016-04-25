/**
 * Class: TileEnvironment
 * Author: Kyle Morris
 *
 * A TileEnvironment contains a grid of tiles, in which those tiles may possess
 * entities.
*/
#ifndef MORRIS_AIMA_TILEENVIRONMENT_H
#define MORRIS_AIMA_TILEENVIRONMENT_H
#include "Environment.h"
#include "TileEnvironmentState.h"

class EntityFactory;
class TileEnvironment : public Environment {
private:
    TileEnvironmentState *state;
    EntityFactory *entityFactory; //for building entities in a Tile Environment
public:
    TileEnvironment();
    ~TileEnvironment();

    virtual void loadEnvironment(string fileName);

    virtual EnvironmentState *readState();

    virtual bool add(Entity *e, Location *place);

    virtual Entity *remove(int id);

    virtual bool exists(int id);

    virtual std::vector<Entity *> getEntities();

    virtual std::string outputToJson() override;

protected:
    /**
     * These are all dummy methods, they don't do anything for a TileEnvironment, but should be overriden.
     * They are not pure virtual, as I'm wanting to be able to initialize a tile environment for the sake of testing purposes.
     */
    virtual void act() override;

    virtual void generate() override;

    virtual void updateResults() override;
};

#endif //MORRIS_AIMA_TILEENVIRONMENT_H
