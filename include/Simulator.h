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
#include "SimulatorResult.h"
#include <QtCore/qtimer.h>
#include "Environment.h"
#include "Visualizer.h"

class Simulator : public QObject {
    Q_OBJECT
private:

    /**
     * How many cycles to perform in the environment. Default is 1.
     */
    long maxCycles = 1;
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

    /**
     * Whether or not simulation has been completed.
     * Replace with asynchronous approach in the future. For now just use this for
     * polling the simulator.
     */
    bool complete = false;
private:
    /**
     * Run a simulation cycle. For a Simulator this will run a cycle() in the TileEnvironment
     * and update the Visualizer display.
     */
    void cycle();
public:
    /**
     * Construct delegates the other constructors, ensuring a stable construction is done even
     * if only the default constructor is initially used.
     */
    void Construct(Environment *e, Visualizer *v, long cycleTime);
    Simulator();
    Simulator(Environment *e, Visualizer *v, long cycleTime);
    virtual ~Simulator();
    //=========GETTERS AND SETTERS=============
    static long getCurrentCycle();

    long getMaxCycles() const;

    long getCycleTime() const;

    void setCycleTime(long cycleTime);

    Environment *getEnvironment() const;

    Visualizer *getDisplay() const;

    /**
     * Begin running the simulation/start timer.
     */
    void start(int numCycles);

    /**
     * Stop running simulation cycles.
     */
    void stop();

    /**
     * Save the results of the simulation.
     */
    void save();

public slots:
    /**
     * Slot for receiving call to timer, ensuring a steady simulation is kept.
     */
    void TimerSlot();
};


#endif //MORRIS_AIMA_TILESIMULATOR_H
