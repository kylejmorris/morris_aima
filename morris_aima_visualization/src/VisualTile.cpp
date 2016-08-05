#include "VisualTile.h"
#include <QPainter>
#include <QGraphicsScene>

QRectF VisualTile::boundingRect() const {
    return QRectF(0,0,RELATIVE_SIZE,RELATIVE_SIZE); //internal coordinate system (0-100)
}

void VisualTile::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    painter->setBrush(Qt::white);
    painter->drawRect(this->boundingRect());

    for(int curr=0; curr<contents.size(); curr++) {
        VisualEntity *currEntity = contents.at(curr);
        if(currEntity!=NULL) {
            currEntity->paint(painter, option, widget);
        }
    }

}

bool VisualTile::addEntity(VisualEntity *target) {
    contents.push_back(target);
    this->scene()->addItem(target);
}

VisualTile::VisualTile() {
    contents.clear();
}

void VisualTile::clean() {
    for(int current=0; current<contents.size(); current++) {
        VisualEntity *entity = contents.at(current);
        this->scene()->removeItem(entity);
        delete entity;
    }

    this->contents.clear();
}
