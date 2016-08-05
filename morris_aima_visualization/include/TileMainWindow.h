/**
 * class: TileMainWindow
 * author: Kyle Morris
 * date: July 30, 2016
 * Description: QT window for display tile environment.
 *
 * This grid is fixed size, meaning it's width/height is specified once and then simulation may be done. Must be reset() to
 * load in a new size.
 */
#ifndef MORRIS_AIMA_VISUALIZATION_TILEMAINWINDOW_H
#define MORRIS_AIMA_VISUALIZATION_TILEMAINWINDOW_H
#include <QMainWindow>
#include <QGraphicsView>
#include <QGraphicsScene>
#include <morris_aima_msgs/TileEnvironmentInfo.h>
#include "VisualTileGrid.h"

class TileMainWindow : public QMainWindow {
    Q_OBJECT
private:
    /**
     * Whether or not display will accept updates and show them.
     */
    bool updatingEnabled = false;

    const int FRAME_WIDTH = 700;
    const int FRAME_HEIGHT = 700;
    QString frameTitle = "Tile Visualizer";

    /**
     * The grid we are displaying
     */
    VisualTileGrid *grid;

    /*
     * Text showing performance measurement reading.
     */
    QGraphicsTextItem *performanceMeasureText;
    /**
    * The embodiement of the environment display, what is contained within it to be rendered.
    */
    QGraphicsScene *scene = NULL;

    /**
     * QT object for the view, that actually displays the scene.
     */
    QGraphicsView *view = NULL;
public:
    TileMainWindow(int argc, char **argv, QWidget *parent);

    /**
     * Display a window with no info yet. Will just set up the frame that will hold environment once loaded.
     * Anything such as a frame to show a border around environment, empty statistics, and other decoration can be loaded at this time.
     * @returns: True if success, false if gui could not be loaded
     */
    bool initialize();
public Q_SLOTS:
    /**
     * Close GUI window.
     */
    void closeWindow();

    /**
     * Update the display with new environment state.
     */
    void update(const morris_aima_msgs::TileEnvironmentInfo &msg);

    /**
     * Enable updates, so the GUI will show changes to environment.
     */
    void enableUpdating();

     /**
     * Disable updates, so the GUI will not show any changes to environment.
     */
    void disableUpdating();
    /**
     * Set the main parameters for display. These must be set before updating/display can take place.
     * PRECONDITIONS: Window must be in initialized state, or reset, cannot set parameters when updating is enabled.
     * @param width: The width in columns of grid to show
     * @param heigth: Height in rows of grid to show
     * @param name: The frames name when loaded.
     */
    void setParameters(int width, int height);

    /**
     * Reset the window to a blank display again, now able to set new parameters and load environment info.
     * PRECONDITIONS: updating must be disabled for display.
     */
    void resetWindow();
    /**
     * Callback for subscription to environment info. Will send a render() signal to main QT window.
     */
    //void render_callback(morris_aima_msgs::TileEnvironmentInfo &msg);
};


#endif //MORRIS_AIMA_VISUALIZATION_TILEMAINWINDOW_H
