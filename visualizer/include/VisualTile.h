/**
* CLASS: VisualTile
* DATE: 13/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Contains a drawable entity, within a nice drawn box... thing?
*/

#ifndef MORRIS_AIMA_VISUALTILE_H
#define MORRIS_AIMA_VISUALTILE_H

#include <QtWidgets/qgraphicsitem.h>

class VisualEntity;
class VisualTile : public QGraphicsItem {
    int x,y; //coordinates of tile relative to grid it is contained in
private:
    virtual QRectF boundingRect() const;
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    std::vector<VisualEntity *>contents;
};

#endif //MORRIS_AIMA_VISUALTILE_H
