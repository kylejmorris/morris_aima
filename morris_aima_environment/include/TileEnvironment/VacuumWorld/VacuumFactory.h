/**
* CLASS: VacuumFactory
* DATE: 25/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for creating objects in VacuumWorld
*/
#ifndef MORRIS_AIMA_VACUUMFACTORY_H
#define MORRIS_AIMA_VACUUMFACTORY_H
#include <EntityFactory.h>
#include <XmlRpcValue.h>

class VacuumFactory : public EntityFactory {
public:
    virtual Entity *createEntity(std::string name, Json::Value properties) override; //DEPRECATED
    virtual Entity *createEntity(std::string name, XmlRpc::XmlRpcValue *properties);
};

#endif //MORRIS_AIMA_VACUUMFACTORY_H
