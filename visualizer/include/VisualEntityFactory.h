/**
* CLASS: VisualEntityFactory
* DATE: 19/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Factory for building Visual Entities for visualizer.
*/

#ifndef MORRIS_AIMA_VISUALENTITYFACTORY_H
#define MORRIS_AIMA_VISUALENTITYFACTORY_H


class VisualEntityFactory {
public:
    void createEntity(std::string type, std::string jsonProperties);
};


#endif //MORRIS_AIMA_VISUALENTITYFACTORY_H
