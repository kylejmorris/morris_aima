/**
* CLASS: VisualEntity
* DATE: 13/04/16
* AUTHOR: ${AUTHOR}
* DESCRIPTION: ${DESCRIPTION}
*/
//
// Created by votick on 13/04/16.
//

#ifndef MORRIS_AIMA_VISUALENTITY_H
#define MORRIS_AIMA_VISUALENTITY_H
#include <QGraphicsItem>


class VisualEntity : public QGraphicsItem {
public:
    virtual QRectF boundingRect() const;

    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};


#endif //MORRIS_AIMA_VISUALENTITY_H
