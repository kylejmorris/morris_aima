#include "FrameVisualizer.h"
#include <string>
#include <QApplication>
//TODO get QT working and QTGraphicsScene

bool FrameVisualizer::load(std::string state) {
    return false;
}


void FrameVisualizer::render() {
    this->scene->addRect(5,5,50,50);
}

FrameVisualizer::FrameVisualizer() {
}

FrameVisualizer::FrameVisualizer(int width, int height, std::string name) {
    Construct(width, height, name);
}

void FrameVisualizer::Construct(int width, int height, std::string name) {
    int dummyArgc = 0;
    char *dummyArgv[1];
    dummyArgv[0] = "null";

    this->frameWidth = width;
    this->frameHeight = height;
    this->frameName = name;

    //initialize the QT gui stuff
    this->scene = new QGraphicsScene();
    this->view = new QGraphicsView(this->scene);
    this->view->resize(this->frameWidth, this->frameHeight);
    this->view->setWindowTitle(this->frameName.c_str());
    this->view->show();
    this->timer = new QTimer();
}

FrameVisualizer::~FrameVisualizer() {
    delete this->view;
    delete this->scene;
    delete this->application;
}

