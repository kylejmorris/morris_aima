/**
* CLASS: VisualImageEntity
* DATE: 28/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Allow for visual item that displays an image
*/
#ifndef MORRIS_AIMA_VISUALIMAGEENTITY_H
#define MORRIS_AIMA_VISUALIMAGEENTITY_H
#include <QtWidgets/qgraphicsitem.h>
#include "VisualEntity.h"

class VisualImageEntity : public VisualEntity {
private:
    const std::string IMAGE_DIRECTORY = "images/";
    std::string imageName;

    /**
     * The actual image item to display.
     */
    QGraphicsPixmapItem *image;

    /**
     * How much to scale image when drawn, between 0 and 1
     */
    double scaleFactor;

    /**
     * Amount to move entity away from it's origin, x/y amounts
     */
    int moveX = 0;
    int moveY = 0;
public:
    /**
     * Feed in image name (with extension).
     */
    VisualImageEntity(std::string imageName);
    /**
     * Move an entity off it's main origin
     * @param x,y: The x and y amounts to move the visual entity, must be between 0 and 50
     */
    virtual void moveBy(int x, int y);

    /**
     * Scale image relative to tile it's in
     * @param amount: the factor to scale entity by. Between 0 and 1
     */
    virtual void scale(double amount);

    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) override;
    virtual ~VisualImageEntity();
};

#endif //MORRIS_AIMA_VISUALIMAGEENTITY_H
