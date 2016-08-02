#include "VisualTile.h"
#include <QPainter>
#include <QGraphicsScene>

QRectF VisualTile::boundingRect() const {
    return QRectF(0,0,RELATIVE_SIZE,RELATIVE_SIZE); //internal coordinate system (0-100)
}

void VisualTile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::white);
    painter->drawRect(this->boundingRect());
//TODO: SOMEHOW THIS BLOCK BELOW, BREAKS MY MAIN GUI. IF UNCOMMENTED THEN THE MAIN GUI WILL FREEZE UP, OTHERWISE AN EMPTY GRID RENDERS JUST FINE. Wtf
   /* for(VisualEntity *currEntity : contents) {
        if(currEntity!=NULL) {
            scene()->addItem(currEntity);
            //currEntity->paint(painter, option, widget);
        }
    }*/
}

bool VisualTile::addEntity(VisualEntity *target) {
    contents.push_back(target);
    this->scene()->addItem(target);
}

VisualTile::VisualTile() {
}

void VisualTile::clean() {
    for(VisualEntity *entity : contents) {
        this->scene()->removeItem(entity);
        delete entity;
    }
    this->contents.clear();
}
