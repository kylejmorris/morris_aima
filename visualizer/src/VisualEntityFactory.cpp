//
// Created by gendo on 19/04/16.
//

#include <bits/stringfwd.h>
#include "VisualEntityFactory.h"
#include "VisualEntity.h"


void VisualEntityFactory::createEntity(std::string type, std::string jsonProperties) {
    VisualEntity *result;
    if(type.compare("dirt")) {
    } else if(type.compare("vacuum")) {

    }
}
