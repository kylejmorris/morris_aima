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
    /**
     * Whether or not a state has been loaded into environment.
     * If some configuration has been set, we are able to publish/cycle/etc.
     */
    bool loaded = false;
    TileEnvironmentState *state;
    EntityFactory *entityFactory; //for building entities in a Tile Environment
public:
    TileEnvironment();
    ~TileEnvironment();

    virtual void loadEnvironment(string fileName);

    /**
     * Load parameters from ros param server and setup the environment.
     * Environment must current NOT be activated(cycling) in order to load a new
     * configuration.
     * @return bool: True if loading was successful. False otherwise.
     */
    virtual bool load();

    virtual EnvironmentState *readState();

    virtual bool add(Entity *e, Location *place);

    virtual Entity *remove(int id);

    virtual bool exists(int id);

    virtual std::vector<Entity *> getEntities();

    virtual std::string outputToJson() override;

    virtual std::vector<Entity *> getEntitiesAt(TileLocation *place);

    /**
     * Find location of an Entity with specified id.
     * @return TileLocation: The location in which entity was found, NULL if it doesn't exist
     */
    virtual TileLocation *getLocationOf(int id);
protected:
    /**
     * These are all dummy methods, they don't do anything for a TileEnvironment, but should be overriden.
     * They are not pure virtual, as I'm wanting to be able to initialize a tile environment for the sake of testing purposes.
     */
    virtual void act() override;

    virtual void generate() override;

    virtual void updateResults() override;

    /**
     * Check if the loaded member is set.
     */
    bool isLoaded();

    virtual void reset();
};


#endif //MORRIS_AIMA_TILEENVIRONMENT_H
