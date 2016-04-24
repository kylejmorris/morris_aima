#include <iostream>
#include "Simulator.h"
#include "TileEnvironment.h"
#include "FrameVisualizer.h"
#include <QDebug>
#include <TileFrameVisualizer.h>
#include <VacuumEnvironment.h>

static int counter = 0;
//TODO could probably use a factory to support generating a simulator object
//TODO fix some weird behaviour if you do AxB sized grid where A is very large relative to B
Simulator::Simulator() {
    //set default incase nothing is specified
    Construct(new VacuumEnvironment(), new TileFrameVisualizer(10,10,"Small World"), 1000);
}

void Simulator::cycle() {
    qDebug() << "cycle running...";
    counter++;
    if(counter>3) {
        environment->cycle();
    }
    std::string environmentState = this->environment->outputToJson();
    this->display->update(environmentState);
    this->display->render();
}

Simulator::Simulator(Environment *e, Visualizer *v, long cycleTime) {
    Construct(e,v,cycleTime);
}

Simulator::~Simulator() {
    delete this->environment;
    delete this->display;
    delete this->timer;
}

void Simulator::Construct(Environment *e, Visualizer *v, long cycleTime) {
    this->environment = e;
    this->display = v;
    this->cycleTime = cycleTime;
    this->timer = new QTimer();
    connect(this->timer, SIGNAL(timeout()), this, SLOT(TimerSlot()));
}

void Simulator::start() {
    this->timer->start(cycleTime);
}

void Simulator::TimerSlot() {
    this->cycle();
}
