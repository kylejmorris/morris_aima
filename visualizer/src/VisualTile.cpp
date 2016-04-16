//
// Created by votick on 13/04/16.
//

#include "VisualTile.h"
#include <QPainter>
#include <QGraphicsScene>

QRectF VisualTile::boundingRect() const {
    return QRectF(0,0,RELATIVE_SIZE,RELATIVE_SIZE); //internal coordinate system (0-100)
}

void VisualTile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::white);
    painter->drawRect(this->boundingRect());
//TODO fix tile display bug top left is showing some NULL text
    /*for(auto currEntity : contents) {
        currEntity->paint(painter,option,widget);
    }*/
}

bool VisualTile::addEntity(VisualEntity *target) {
    contents.push_back(target);
    this->scene()->addItem(target);
}

VisualTile::VisualTile() {
}
