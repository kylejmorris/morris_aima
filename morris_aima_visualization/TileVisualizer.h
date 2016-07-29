/**
 * class: TileVisualizer
 * author: Kyle Morris
 * date: July 28, 2016
 * Description: Will handle displaying of a 2D grid, connecting the QT window and the RosNode associated.
 *
 * This grid is fixed size, meaning it's width/height is specified once and then simulation may be done. Must be reset() to
 * load in a new size.
 */
#ifndef MORRIS_AIMA_VISUALIZATION_TILEVISUALIZER_H
#define MORRIS_AIMA_VISUALIZATION_TILEVISUALIZER_H

class TileVisualizer {
private:
    /**
     * The Main window containing the grid to display
     */
    TileMainWindow *window;

    /**
     * The ros node handling signals for ROS, communicate with environment/etc.
     */
    TileQRosNode qnode;

public:
    TileVisualizer(int argc, char **argv);

    /**
     * Set up sockets and signals between QT and ROS
     */
    bool initialize();

    /**
     * Begin running ROS and QT on separate threads, just the main loops.
     */
    bool run();
};


#endif //MORRIS_AIMA_VISUALIZATION_TILEVISUALIZER_H
