/**
* CLASS: TileSimulator
* DATE: 31/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Simulate anything taking place on Tile environments. A main loop will be called from here
* which will run an environment cycle, then update the Visualiser.
*/
#ifndef MORRIS_AIMA_TILESIMULATOR_H
#define MORRIS_AIMA_TILESIMULATOR_H
#include <QObject>
#include <QtCore/qtimer.h>
#include "Environment.h"
#include "Visualizer.h"

class Simulator : public QObject {
    Q_OBJECT
private:

    /**
     * How long a given cycle of environment should take, that is to process environment/display.
     * Measured in millisec
     */
    long cycleTime = 1000;

    /**
     * Will handle cycle timing. Each iteration will signal a full simulator loop.
     */
    QTimer *timer;

    /**
     * The environment being simulated.
     */
    Environment *environment;

    /**
     * The display being used to show simulation.
     */
    Visualizer *display;

public:
    /**
     * Construct delegates the other constructors, ensuring a stable construction is done even
     * if only the defaultconstructor is initially used.
     */
    void Construct(Environment *e, Visualizer *v, long cycleTime);

    Simulator();
    Simulator(Environment *e, Visualizer *v, long cycleTime);
    virtual ~Simulator();

    /**
     * Begin running the simulation/start timer.
     */
    void start();

    /**
     * Run a simulation cycle. For a Simulator this will run a moment() in the TileEnvironment
     * and update the Visualizer display.
     */
    void cycle();

public slots:

    /**
     * Slot for receiving call to timer, ensuring a steady simulation is kept.
     */
    void TimerSlot();
};


#endif //MORRIS_AIMA_TILESIMULATOR_H
