/**
* CLASS: Entity
* DATE: 04/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: An Entity exists in an environment, as some sort of "thing".
*/
#ifndef MORRIS_AIMA_ENTITY_H
#define MORRIS_AIMA_ENTITY_H
#include <string>


class Entity {
private:
    static int UNIQUE_ID;
    /**
     * Unique id for this entity. Starts at 0 and increments for each new entity created
     * using static variable in this class.
     */
    int id;
public:
    /**
     * Default constructor that will increment id counter.
     */
    Entity();
    ~Entity(){};
    /**
     * Compare this entity with another one.
     * @param: Entity *other: The entity to compare to
     * @return: <0 if this entity is logically less than the other
     * > 0 if this entity is logically greater than the other
     * 0 means both entities are logically equivalent.
     */
    virtual int compareTo(Entity *other) = 0;

    /**
     * Convert the Entity into some readable string format.
     */
    virtual std::string toString() = 0;

    int getId();

};

#endif //MORRIS_AIMA_ENTITY_H
