#include <iostream>
#include "Simulator.h"
#include "TileEnvironment.h"
#include "FrameVisualizer.h"
#include <QDebug>
#include <TileFrameVisualizer.h>
#include <VacuumEnvironment.h>

static int currentCycle = 0;
//TODO could probably use a factory to support generating a simulator object
//TODO fix some weird behaviour if you do AxB sized grid where A is very large relative to B
Simulator::Simulator() {
    //set default in case nothing is specified
    Construct(new VacuumEnvironment(), new TileFrameVisualizer(1,2,"Small World"), 1000);
}

void Simulator::cycle() {
    qDebug() << "cycle running...";

    std::string environmentState = this->environment->outputToJson();
    this->display->update(environmentState);
    this->display->render();
    currentCycle++;
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
    std::string environmentState = this->environment->outputToJson();
    this->display->update(environmentState);
    this->display->render();
    connect(this->timer, SIGNAL(timeout()), this, SLOT(TimerSlot()));
}

void Simulator::start(int numCycles) {
    currentCycle = 0;
    this->maxCycles = numCycles;
    this->timer->start(cycleTime);
}

void Simulator::stop() {
    this->timer->stop();
}

void Simulator::TimerSlot() {
    if(currentCycle<maxCycles) {
        this->cycle();
        currentCycle++;
    } else {
        stop();
    }
}
