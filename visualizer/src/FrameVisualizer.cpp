#include "FrameVisualizer.h"
#include <string>
#include <QApplication>
#include <iostream>
void FrameVisualizer::render() {
    this->view->viewport()->update();
}

FrameVisualizer::FrameVisualizer() {
    Construct(this->frameWidth,this->frameHeight,"Frame Visualizer");
}

FrameVisualizer::FrameVisualizer(int width, int height, std::string name) {
    Construct(width, height, name);
}

void FrameVisualizer::Construct(int width, int height, std::string name) {
    this->frameWidth = width;
    this->frameHeight = height;
    this->setFrameName(name);

    //initialize the QT gui stuff
    this->scene = new QGraphicsScene();
    this->view = new QGraphicsView(this->scene);
    this->view->setViewportUpdateMode(QGraphicsView::NoViewportUpdate); //we only want update on render
    this->view->resize(this->frameWidth, this->frameHeight);
    updateFrameTitle();
    this->view->show();
}


FrameVisualizer::~FrameVisualizer() {
    delete this->view;
    delete this->scene;
}

void FrameVisualizer::update(std::string state) {
}

/*========================================
         GETTERS AND SETTERS
 =========================================*/
    QGraphicsScene *FrameVisualizer::getScene() const {
        return scene;
    }

    void FrameVisualizer::setScene(QGraphicsScene *scene) {
        FrameVisualizer::scene = scene;
    }


    std::string FrameVisualizer::getFrameName() const {
        return frameName.toStdString();
    }

    void FrameVisualizer::setFrameName(const std::string &newName) {
        this->frameName = QString::fromStdString(newName);
    }


    int FrameVisualizer::getFrameHeight() const {
        return frameHeight;
    }

    void FrameVisualizer::setFrameHeight(int frameHeight) {
        FrameVisualizer::frameHeight = frameHeight;
    }

    int FrameVisualizer::getFrameWidth() const {
        return frameWidth;
    }

    void FrameVisualizer::setFrameWidth(int frameWidth) {
        FrameVisualizer::frameWidth = frameWidth;
    }

void FrameVisualizer::updateFrameTitle() {
    this->view->setWindowTitle(this->frameName);
}

QGraphicsView *FrameVisualizer::getView() {
    return this->view;
}
