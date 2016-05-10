/**
* CLASS: WallEntity
* DATE: 11/05/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Agents cannot pass through walls.
*/

#ifndef MORRIS_AIMA_WALLENTITY_H
#define MORRIS_AIMA_WALLENTITY_H


#include <Entity.h>

class WallEntity : public Entity {

public:
    virtual std::string toString() override;

    virtual std::string getType() override;

    //walls=walls by default.
    virtual int compareTo(Entity *other) override;
};


#endif //MORRIS_AIMA_WALLENTITY_H
