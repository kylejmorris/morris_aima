#include <bits/stringfwd.h>
#include <VisualShapeEntity.h>
#include <VisualImageEntity.h>
#include "VisualEntityFactory.h"
#include "VisualEntity.h"


VisualEntity * VisualEntityFactory::createEntity(std::string type, std::string jsonProperties) {
    VisualEntity *result = NULL;

    if(type.compare("Dirt")==0) {
        VisualImageEntity *temp = new VisualImageEntity("dirt.png");
        //TODO this part is goofed due to how coordinates are handled. fix sometime
        temp->moveBy(100,100); //place it in bottom right corner
        temp->moveBy(100,100); //place it in bottom right corner
        temp->moveBy(40,40); //place it in bottom right corner
        temp->scale(0.5);
        result = temp;
    } else if(type.compare("VacuumAgent")==0) {
        VisualImageEntity *temp = new VisualImageEntity("vacuum.png");
        temp->scale(0.5);
        result = temp;
    }

    return result;
}
