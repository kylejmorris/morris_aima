/**
* CLASS: EntityFactory
* DATE: 24/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for creating entities in an abstract environment. This is the abstract class for what a
* factory must do.
*/

#ifndef MORRIS_AIMA_ENTITYFACTORIY_H
#define MORRIS_AIMA_ENTITYFACTORIY_H
#include <string>
#include <json/json.h>
#include <XmlRpcValue.h>

class Entity;

class EntityFactory {
public:
    /**
     * Create an entity given the Entities name and the properties associated with it. The entities creatable will
     * depend on the Environment we are simulating.
     * @param name: String representing entities name to create
     * @param Json::Value: Json object for properties associated with entity we're making.
     * @return Entity: The entity created
     */
    virtual Entity  *createEntity(std::string name, Json::Value properties) = 0;

    /**
     *
     * Create an entity given the Entities name and the properties associated with it. The entities creatable will
     * depend on the Environment we are simulating.
     * @param name: String representing entities name to create
     * @param properties: properties associated with entity we're making.
     * @return Entity: The entity created
     */
    virtual Entity *createEntity(std::string name, XmlRpc::XmlRpcValue *properties) =0;
};

#endif //MORRIS_AIMA_ENTITYFACTORIY_H
