/**
* CLASS: TileGrid
* DATE: 13/04/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Grid to be displayed by the visualizer. Contains drawable Entities.
*/
#ifndef MORRIS_AIMA_VISUAL_TILEGRID_H
#define MORRIS_AIMA_VISUAL_TILEGRID_H
#include <QtWidgets/qgraphicsitem.h>
#include "VisualTile.h"
#include <vector>

class VisualTileGrid : public QGraphicsItem {
private:
//Width and height of grid. Once set this doesn't change. This is the actual grids coordinate system/bounding rect
    const int WIDTH;
    const int HEIGHT;

    /**
     * Whether or not tiles have been drawn. this is set to false until we draw all the initial tiles.
     * Reason being is we dont' want to have to manually iterate through all the grid and redraw for every frame.
     * It's easier to just have the graphics scene/view detect changes and treat each tile as some object within the scene
     * and not a block we must handle within the VisualTileGrid. This grid logically contains VisualTile objects, but the
     * scene and view contain the actual pointers to them to manage efficiently.
     */
    bool drawn = false;
    //The number of row/columns in a row*column grid, not to be confused with width & height.
    int rows;
    int columns;
    std::vector<std::vector<VisualTile *>> tiles;
public:
    VisualTileGrid(int rows, int cols, int width, int height);
    virtual QRectF boundingRect() const;
    virtual void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);

    /**
     * Paint the individual tiles within grid. We do this separately from the grid itself, so the grid
     * isn't constantly being redrawn.
     */
    void paintTiles(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
};


#endif //MORRIS_AIMA_VISUAL_TILEGRID_H
