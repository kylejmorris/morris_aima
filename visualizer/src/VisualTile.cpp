//
// Created by votick on 13/04/16.
//

#include "VisualTile.h"
#include <QPainter>

QRectF VisualTile::boundingRect() const {
    return QRectF(50,0,100,100);
}

void VisualTile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::yellow);
    painter->drawRect(this->boundingRect());
}
