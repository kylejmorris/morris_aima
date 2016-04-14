#include "FrameVisualizer.h"
#include <string>
#include <QApplication>
#include <iostream>

void FrameVisualizer::render() {
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
    this->frameName = name;

    //initialize the QT gui stuff
    this->scene = new QGraphicsScene();
    this->view = new QGraphicsView(this->scene);
    this->view->resize(this->frameWidth, this->frameHeight);
    this->view->setWindowTitle(this->frameName.c_str());
    this->view->show();
}

FrameVisualizer::~FrameVisualizer() {
    delete this->view;
    delete this->scene;
}


void FrameVisualizer::update(std::string state) {
}
