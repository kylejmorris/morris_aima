/**
* CLASS: VisualTile
* DATE: 13/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Contains a drawable entity, within a nice drawn box... thing?
*/

#ifndef MORRIS_AIMA_VISUALTILE_H
#define MORRIS_AIMA_VISUALTILE_H

#include <QtWidgets/qgraphicsitem.h>
#include "VisualEntity.h"
#include <vector>

class VisualTile : public QGraphicsItem {
public:
    static const int RELATIVE_SIZE = 100; //tile is a box, we know that. This is it's relative coordinate scale, (0,0) is top left, (100,100) is bottom right

private:
    std::vector<VisualEntity *> contents;
public:
    VisualTile();
    virtual QRectF boundingRect() const;
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    bool addEntity(VisualEntity *entity);

    /**
     * Clean an individual tile, removing the contents from it and from the graphics scene.
     */
    void clean();
};

#endif //MORRIS_AIMA_VISUALTILE_H
