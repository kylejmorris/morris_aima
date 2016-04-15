//
// Created by votick on 13/04/16.
//

#include "VisualTile.h"
#include <QPainter>

QRectF VisualTile::boundingRect() const {
    return QRectF(0,0,RELATIVE_SIZE,RELATIVE_SIZE); //internal coordinate system (0-100)
}

void VisualTile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::white);
    painter->drawRect(this->boundingRect());
}

VisualTile::VisualTile() {
}
