//
// Created by gendo on 19/04/16.
//

#include <bits/stringfwd.h>
#include <VisualShapeEntity.h>
#include "VisualEntityFactory.h"
#include "VisualEntity.h"


VisualEntity * VisualEntityFactory::createEntity(std::string type, std::string jsonProperties) {
    VisualEntity *result;
    if(type.compare("dirt")) {
        result = new VisualShapeEntity;
    } else if(type.compare("vacuum")) {
        result = new VisualShapeEntity;
    }
    return result;
}
