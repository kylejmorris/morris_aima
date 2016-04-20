/**
* CLASS: FrameVisualizer
Arash.info & DATE: 29/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A frame is some GUI/box with contents in it displayable to user. Not something like a console/etc
*/
#ifndef MORRIS_AIMA_FRAMEVISUALIZER_H
#define MORRIS_AIMA_FRAMEVISUALIZER_H
#include <string>
#include "Visualizer.h"
#include <QApplication>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QtCore/qtimer.h>

class FrameVisualizer : public Visualizer{
protected:
    /**
     * Frame properties and default values.
     */
    int frameWidth = 1000;
    int frameHeight = 1000;

    QString frameName = "Frame Visualizer";

private:
    /**
     * The embodiement of the environment display, what is contained within it to be rendered.
     */
    QGraphicsScene *scene;

    /**
     * QT object for the view, that actually displays the scene.
     */
    QGraphicsView *view;

public:
    //default constructor
    FrameVisualizer();
    FrameVisualizer(int width, int height, std::string);

    /**
     * Main constructor, which will be delegated to by the default constructor/etc.
     * The actual parameters are set here. Other constructors allow for a user to not
     * specify certain parameters, and instead have them defaulted here.
     */
    virtual void Construct(int width, int height, std::string name);

    virtual ~FrameVisualizer();

    virtual void update(std::string state);

    /**
     * Update the view with changes made.
     */
    void render();

    /**
     * Update the displayed gui title if the frame name has been changed. This isn't to be confused with the frameName
     * which can be changed at any point. The gui must still be updated.
     */
    void updateFrameTitle();
/*========================================
         GETTERS AND SETTERS
 =========================================*/
    QGraphicsScene *getScene() const;

    void setScene(QGraphicsScene *scene);

    std::string getFrameName() const;

    void setFrameName(const std::string &newName);

    int getFrameHeight() const;

    void setFrameHeight(int frameHeight);

    int getFrameWidth() const;

    void setFrameWidth(int frameWidth);

    QGraphicsView *getView();
};

#endif //MORRIS_AIMA_FRAMEVISUALIZER_H
