#include "VisualShapeEntity.h"
#include <QPainter>

void VisualShapeEntity::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::blue); //default color for now //TODO make this customizable
    this->setZValue(Z_VALUE_PRIORITY);
    painter->drawRect(this->boundingRect());
}
