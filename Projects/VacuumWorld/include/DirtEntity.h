/**
* CLASS: DirtEntity
* DATE: 27/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Dirt that may be scattered through an environment
*/

#ifndef MORRIS_AIMA_DIRTENTITY_H
#define MORRIS_AIMA_DIRTENTITY_H
#include "Entity.h"

class DirtEntity : public Entity {
public:
    virtual int compareTo(Entity *other) override;

    virtual std::string toString() override;

    virtual std::string getType() override;
};

#endif //MORRIS_AIMA_DIRTENTITY_H
