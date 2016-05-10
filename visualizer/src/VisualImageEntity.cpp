#include "VisualImageEntity.h"
#include <QPixmap>
#include <QImage>
#include <QPainter>
#include <QStyleOptionGraphicsItem>
#include <QWidget>
#include <QtCore/qdir.h>

VisualImageEntity::VisualImageEntity(std::string imageName) {
    std::string imagePath = IMAGE_DIRECTORY + imageName;
    this->image = new QGraphicsPixmapItem(QPixmap(imagePath.c_str()), this);
}

void VisualImageEntity::scale(double amount) {
    if(amount>0 && amount<=1.0) {
        this->scaleFactor = amount;
    }
}


void VisualImageEntity::moveBy(int x, int y) {
    this->moveX+=x;
    this->moveY+=y;
}

VisualImageEntity::~VisualImageEntity() {
    delete this->image;
}

void VisualImageEntity::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    this->setZValue(Z_VALUE_PRIORITY);
    VisualEntity::moveBy(moveX+5,moveY+5); //the +5 is to offset border of tile image is on
    this->setScale(VisualEntity::scale()*this->scaleFactor);
    painter->drawRect(this->boundingRect());
}
