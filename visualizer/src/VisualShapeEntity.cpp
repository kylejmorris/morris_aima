#include "VisualShapeEntity.h"
#include <QPainter>

VisualShapeEntity::VisualShapeEntity() {
    this->itemColor = DEFAULT_ITEM_COLOR;
}

VisualShapeEntity::VisualShapeEntity(QColor color) {
    this->itemColor = color;
}

void VisualShapeEntity::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(this->itemColor);
    this->setZValue(Z_VALUE_PRIORITY);
    painter->drawRect(this->boundingRect());
}
