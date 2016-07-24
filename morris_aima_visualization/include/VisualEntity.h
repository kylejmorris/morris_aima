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
protected:
    /**
     * The priority of item when drawn, the higher it is the closer to front of view it will be rendered.
     */
    const int Z_VALUE_PRIORITY = 3;

    /**
     * What we display as text for initialization of abstract VisualEntity
     */
    const std::string DEFAULT_TEXT = "NULL ENTITY";
public:
    /**
     * Used to return outer boundary of visual entity, it's the same for all children so not overriden.
     */
    QRectF boundingRect() const;

    /**
     * Each visual entity decides how to paint itself.
     */
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};


#endif //MORRIS_AIMA_VISUALENTITY_H
