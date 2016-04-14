#include <QtCore/qrect.h>
#include <QtGui/qpainter.h>
#include <QtWidgets/qstyleoption.h>
#include <VisualTileGrid.h>
#include "TileGrid.h"

VisualTileGrid::VisualTileGrid(int width, int height) : WIDTH(width), HEIGHT(height) {
}

QRectF VisualTileGrid::boundingRect() const {
    return QRectF(0,0,WIDTH,HEIGHT);
}

void VisualTileGrid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
    painter->setBrush(Qt::lightGray);
    painter->drawRect(this->boundingRect());
}
