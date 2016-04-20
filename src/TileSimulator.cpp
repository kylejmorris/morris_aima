#include <iostream>
#include "TileSimulator.h"
#include "TileEnvironment.h"
#include "FrameVisualizer.h"
#include <QDebug>
#include <TileFrameVisualizer.h>

static int counter = 0;
//TODO fix some weird behaviour if you do AxB sized grid where A is very large relative to B
TileSimulator::TileSimulator() {
    //set default incase nothing is specified
    Construct(new TileEnvironment(), new TileFrameVisualizer(2,1,"Small World"), 1000);
}

void TileSimulator::cycle() {
    qDebug() << "cycle running...";
    counter++;
    if(counter>3) {
        this->display->update("nil");
    }
    this->display->render();
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
