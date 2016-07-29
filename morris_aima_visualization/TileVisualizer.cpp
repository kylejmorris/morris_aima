#include <QtCore/QArgument>
#include <QtCore/QObject>
#include "TileVisualizer.h"
#include "morris_aima_msgs/TileEnvironmentInfo.h"

TileVisualizer::TileVisualizer(int argc, char *argv) : window(argc, argv) qnode(argc,argv) {
}

bool TileVisualizer::initialize() {
    /**
     * Showing gui with no info about environment yet
     */
    this->window->initialize();
    /**
     * Setting up ros services/subscriptions and publications.
     */
    this->qnode->initialize();

    /**
     * SIGNALS/SLOTS
     */
    QObject::connect(this->qnode, SIGNAL(rosShutdown()), this->window, SLOT(Close()));
    //Signal updating the display when ros gets new info
    QObject::connect(this->qnode, SIGNAL(render(morris_aima_msgs::TileEnvironmentInfo)), this->window, SLOT(render(morris_aima_msgs::TileEnvironmentInfo)));

    QObject::connect(this->qnode, SIGNAL(start()), this->window, SLOT(start())); //Window gui will now accept updates and display most recent info

    QObject::connect(this->qnode, SIGNAL(setParameters(int,int,std::string)), this->window, SLOT(setParameters(int,int,std::string))); //Load initial params such as grid width/height and frame name. These don't change during simulation.

    QObject::connect(this->qnode, SIGNAL(reset()), this->window, SLOT(resetWindow())); //reset to blank window, able to load new grid sizes now.
    return true;
}

bool TileVisualizer::run() {
    this->qnode->run();
    this->window->run();
    return true;
}

