/**
* CLASS: VisualBlockEntity
* DATE: 19/04/16
* AUTHOR: ${AUTHOR}
* DESCRIPTION: A drawn shape entity, with some simple color properties/filling options. Drawn using painter tool.
*/
#ifndef MORRIS_AIMA_VISUALBLOCKENTITY_H
#define MORRIS_AIMA_VISUALBLOCKENTITY_H
#include "VisualEntity.h"

class QRectF;
class QPainter;
class QStyleOptionGraphicsItem;
class QWidget;
class QColor;

class VisualShapeEntity : public VisualEntity {
    const QColor DEFAULT_ITEM_COLOR = Qt::blue;
    //TODO add ability to customize color, make enums for handling color types/other properties of shape
    QColor itemColor;

public:
    VisualShapeEntity();
    VisualShapeEntity(QColor color);

    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
};


#endif //MORRIS_AIMA_VISUALBLOCKENTITY_H
