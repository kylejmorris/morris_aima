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

    std::string frameName = "Frame Visualizer";

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

    virtual /**
     * Main constructor, which will be delegated to by the default constructor/etc.
     * The actual parameters are set here. Other constructors allow for a user to not
     * specify certain parameters, and instead have them defaulted here.
     */
    void Construct(int width, int height, std::string name);

    virtual ~FrameVisualizer();

    virtual void update(std::string state);

    QGraphicsScene *getScene() const {
        return scene;
    }

    void setScene(QGraphicsScene *scene) {
        FrameVisualizer::scene = scene;
    }

    virtual void render();
};

#endif //MORRIS_AIMA_FRAMEVISUALIZER_H