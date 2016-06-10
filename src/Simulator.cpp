#include <iostream>
#include "Simulator.h"
#include "TileEnvironment.h"
#include "FrameVisualizer.h"
#include <QDebug>
#include <TileFrameVisualizer.h>
#include <VacuumEnvironment.h>
#include <SimulatorResultFactory.h>
#include <MandCEnvironment.h>
#include <TerminalTextVisualizer.h>

static long currentCycle = 0;
static int initialDisplay = 10;
//TODO could probably use a factory to support generating a simulator object
//TODO fix some weird behaviour if you do AxB sized grid where A is very large relative to B
Simulator::Simulator() {
    //set default in case nothing is specified
    Construct(new VacuumEnvironment(), new TileFrameVisualizer(1,2,"Small World"), 1000);
}

void Simulator::cycle() {
    qDebug() << "cycle running...";

    this->environment->cycle();
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

void Simulator::start(int numCycles) {
    currentCycle = 0;
    this->maxCycles = numCycles;
    this->timer->start(cycleTime);
}

//TODO figure out a way to signal that simulation is done.
void Simulator::stop() {
    this->timer->stop();
    save();
}

void Simulator::save() {
    //TODO noticing a LOT of factories using the same "VacuumWorld" and other names of projects. Should probably make some global set of variables for the project names so I don't have to manuelly update several factories. Perhaps a root Factory object?
    std::string simulationType = "VacuumWorld";
    if (dynamic_cast<MandCEnvironment *>(this->environment) != NULL) {
        simulationType = "MissionariesAndCannibals";
    }
    SimulatorResult *result = SimulatorResultFactory::createSimulatorResult(simulationType, this);
    result->writeToFile("simulation_output.txt");
}

void Simulator::TimerSlot() {
    if(initialDisplay>0) {
       initialDisplay--;
       std::string environmentState = this->environment->outputToJson();
       this->display->update(environmentState);
       this->display->render();
    } else {
        if (currentCycle < maxCycles) {
            this->cycle();
            currentCycle++;
        } else {
            stop();
        }
    }
}


long Simulator::getMaxCycles() const {
    return this->maxCycles;
}

long Simulator::getCycleTime() const {
    return this->cycleTime;
}

void Simulator::setCycleTime(long cycleTime) {
    this->cycleTime = cycleTime;
}

Environment *Simulator::getEnvironment() const {
    return this->environment;
}

long Simulator::getCurrentCycle() {
    return currentCycle;
}

Visualizer *Simulator::getDisplay() const {
    return this->display;
}
