#include <QtCore/qrect.h>
#include <QGraphicsScene>
#include <QtGui/qpainter.h>
#include <QtWidgets/qstyleoption.h>
#include <VisualTileGrid.h>
#include <sstream>
#include <TileLocation.h>

VisualTileGrid::VisualTileGrid(int cols, int rows, int width, int height) : WIDTH(width), HEIGHT(height) {
    double scaleFactor = (WIDTH*1.0)/(cols*1.0*VisualTile::RELATIVE_SIZE);
    this->rows = rows;
    this->columns = cols;

    for(int row = 0; row<rows; row++) {
        std::vector<VisualTile *> currentRow;
        for(int col=0; col<cols; col++) {
            VisualTile *curr = new VisualTile; //current tile to push back onto row
            currentRow.push_back(curr);
            curr->moveBy((WIDTH/cols)*col,(HEIGHT/rows)*row);
            curr->setScale(scaleFactor);
        }
        tiles.push_back(currentRow);
    }
}

bool VisualTileGrid::addEntity(VisualEntity *entity, TileLocation *location) {
    double scaleFactor = (WIDTH*1.0)/(this->columns*1.0*VisualTile::RELATIVE_SIZE);
    bool result = false; //false until proven otherwise
    int xPos;
    int yPos;

    if (entity != NULL && location != NULL) {
        xPos = location->getX();
        yPos = location->getY();
        //TODO make a method for handling out of bounds check
        //make sure we don't go out of bounds
        if(xPos<this->columns && yPos<this->rows) {
            entity->moveBy((WIDTH / this->columns) * xPos,
                           (HEIGHT / this->rows) * yPos); //moving it by relative tile distnace, based on #tiles in grid
            entity->setScale(scaleFactor);
            this->tiles[yPos][xPos]->addEntity(entity);
            result = true;
        }
    }

    return result;
}

void VisualTileGrid::clean() {
    for(int row=0; row<this->rows; row++) {
        for(int col=0; col<this->columns;col++) {
            tiles[row][col]->clean();
        }
    }
}

QRectF VisualTileGrid::boundingRect() const {
    return QRectF(0,0,WIDTH,HEIGHT);
}

void VisualTileGrid::paint(QPainter *painter, const QStyleOptionGraphicsItem *option,
                           QWidget *widget) {
    std::stringstream out;
    painter->setBrush(Qt::lightGray);
    painter->drawRect(this->boundingRect());

#ifndef NDEBUG //grid size/properties
    QGraphicsTextItem *debugText = new QGraphicsTextItem(this->parentItem());
    out << "DEBUG INFO \n_____\n[" << WIDTH << "x" << HEIGHT << " grid]\nRows: " << this->rows << "\nColumns: " << this->columns << "\n";
    debugText->setPlainText(QString::fromStdString(out.str()));
    debugText->setZValue(4);
    this->scene()->addItem(debugText);

#endif
    this->paintTiles(painter,option,widget);
}

int VisualTileGrid::getTileSize() {
    return WIDTH/std::max(rows,columns);
}

void VisualTileGrid::paintTiles(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget) {
    VisualEntity *currEntity;

    if(!drawn) {
        for (int row = 0; row < this->rows; row++) {
            for (int col = 0; col < this->columns; col++) {
                QGraphicsScene *scene = this->scene();
                if (scene != NULL) {
                    scene->addItem(this->tiles[row][col]);
                }
            }
        }

        //make sure to record that we've drawn the tiles and created memory for them referenced by global Scene
        drawn = true;
    }
}
