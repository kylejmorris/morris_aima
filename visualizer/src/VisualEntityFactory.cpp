#include <bits/stringfwd.h>
#include <VisualShapeEntity.h>
#include <VisualImageEntity.h>
#include "VisualEntityFactory.h"
#include "VisualEntity.h"


VisualEntity * VisualEntityFactory::createEntity(std::string type, int boundingBoxSize, std::string jsonProperties) {
    VisualEntity *result = NULL;
    double moveFactor = (1.0*boundingBoxSize)/100.0;

    if(type.compare("Dirt")==0) {
        VisualImageEntity *temp = new VisualImageEntity("dirt.png");
        temp->scale(0.4*0.25); //fitting it into a given tile, then making it 25% of that tile size
        temp->moveBy(60*moveFactor, 60*moveFactor);
        result = temp;
    } else if(type.compare("VacuumAgent")==0) {
        VisualImageEntity *temp = new VisualImageEntity("vacuum.png");
        temp->scale(0.3);
        result = temp;
    } else if(type.compare("Wall")==0) {
        VisualImageEntity *temp = new VisualImageEntity("wall.png");
        temp->scale(0.4); //exact size of tile
        result = temp;
    }

    return result;
}
