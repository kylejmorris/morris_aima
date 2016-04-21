/**
* CLASS: VisualEntityFactory
* DATE: 19/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for building Visual Entities for visualizer.
*/

#ifndef MORRIS_AIMA_VISUALENTITYFACTORY_H
#define MORRIS_AIMA_VISUALENTITYFACTORY_H

class VisualEntity;
class VisualEntityFactory {
public:
    static VisualEntity *createEntity(std::string type, std::string jsonProperties);
};

#endif //MORRIS_AIMA_VISUALENTITYFACTORY_H
