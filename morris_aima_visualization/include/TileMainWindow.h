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
#include <morris_aima_msgs/TileEnvironmentInfo.h>

class TileMainWindow : public QMainWindow {
    Q_OBJECT
private:
    /**
     * Whether or not display will accept updates and show them.
     */
    bool updatingEnabled = false;
public:
    TileMainWindow(int argc, char **argv);

    /**
     * Display a window with no info yet. Will just set up the frame that will hold environment once loaded.
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
    void update(morris_aima_msgs::TileEnvironmentInfo &msg);

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
    void setParameters(int width, int height, std::string name);

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
