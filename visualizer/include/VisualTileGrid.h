/**
* CLASS: TileGrid
* DATE: 13/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Grid to be displayed by the visualizer. Contains drawable Entities.
*/
#ifndef MORRIS_AIMA_VISUAL_TILEGRID_H
#define MORRIS_AIMA_VISUAL_TILEGRID_H
#include <QtWidgets/qgraphicsitem.h>
#include "VisualTile.h"

class VisualTileGrid : public QGraphicsItem {
private:
//Width and height of grid. Once set this doesn't change.
    const int WIDTH;
    const int HEIGHT;
    std::vector<std::vector<VisualTile>> tiles;
public:
    VisualTileGrid(int width, int height);
    virtual QRectF boundingRect() const;
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};


#endif //MORRIS_AIMA_VISUAL_TILEGRID_H
