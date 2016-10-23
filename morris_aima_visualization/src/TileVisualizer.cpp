#include <QtCore/QArgument>
#include <QtCore/QObject>
#include "TileVisualizer.h"
#include "TileQRosNode.h"
#include "morris_aima_msgs/TileEnvironmentInfo.h"

TileVisualizer::TileVisualizer(int argc, char **argv) : window(argc, argv, NULL), qnode(argc,argv) {
}

bool TileVisualizer::initialize() {
    /**
     * Showing gui with no info about environment yet
     */
    this->window.initialize();
    /**
     * SIGNALS/SLOTS
     */
    QObject::connect(&qnode, SIGNAL(rosShutdown()), &window, SLOT(closeWindow()));
    //Signal updating the display when ros gets new info
    QObject::connect(&qnode, SIGNAL(update(const morris_aima_msgs::TileEnvironmentInfo)), &window, SLOT(update(const morris_aima_msgs::TileEnvironmentInfo)));

    QObject::connect(&qnode, SIGNAL(enableUpdating()), &window, SLOT(enableUpdating())); //Window gui will now accept updates and display most recent info

    QObject::connect(&qnode, SIGNAL(freeze()), &window, SLOT(disableUpdating()));

    QObject::connect(&qnode, SIGNAL(setParameters(int,int)), &window, SLOT(setParameters(int,int))); //Load initial params such as grid width/height and frame name. These don't change during simulation.

    QObject::connect(&qnode, SIGNAL(reset()), &window, SLOT(resetWindow())); //reset to blank window, able to load new grid sizes now.

     /**
     * Setting up ros services/subscriptions and publications.
     */
    this->qnode.initialize();

    return true;
}

bool TileVisualizer::run() {
//    this->qnode.run();
    return true;
}

