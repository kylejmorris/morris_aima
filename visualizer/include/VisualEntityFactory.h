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
    /**
     * The boundingBoxSize refers to the size of the object our entity is being placed into.
     * For example on a grid, a tile that is 100x100, we would put 100 for this value, and our visualentity will be scaled accordingly.
     */
    static VisualEntity *createEntity(std::string type, int boundingBoxSize, std::string jsonProperties);
};

#endif //MORRIS_AIMA_VISUALENTITYFACTORY_H
