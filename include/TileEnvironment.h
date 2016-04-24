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

class TileEnvironment : public Environment {
private:
    TileEnvironmentState *state;
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
};

#endif //MORRIS_AIMA_TILEENVIRONMENT_H
