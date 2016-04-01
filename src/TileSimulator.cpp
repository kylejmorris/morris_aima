#include <iostream>
#include "TileSimulator.h"
#include "TileEnvironment.h"
#include "FrameVisualizer.h"
#include <QDebug>

static int counter = 0;
TileSimulator::TileSimulator() {
    //set default incase nothing is specified
    Construct(new TileEnvironment(), new FrameVisualizer(), 1000);
}

void TileSimulator::cycle() {
    qDebug() << "cycle running...";
    counter++;
    this->display->update("nil");
    if(counter>10) {
        this->display->render();
        counter = 0;
    }
}

TileSimulator::TileSimulator(Environment *e, Visualizer *v, long cycleTime) {
    Construct(e,v,cycleTime);
}

TileSimulator::~TileSimulator() {
    delete this->environment;
    delete this->display;
    delete this->timer;
}

void TileSimulator::Construct(Environment *e, Visualizer *v, long cycleTime) {
    this->environment = environment;
    this->display = v;
    this->cycleTime = cycleTime;
    this->timer = new QTimer();
    connect(this->timer, SIGNAL(timeout()), this, SLOT(TimerSlot()));
}

void TileSimulator::start() {
    this->timer->start(cycleTime);
}

void TileSimulator::TimerSlot() {
    this->cycle();
}
