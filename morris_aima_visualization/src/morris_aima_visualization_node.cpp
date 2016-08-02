#include "ros/ros.h"
#include <QApplication>
#include "std_msgs/String.h"
#include <sstream>
#include <VisualizerFactory.h>
#include <TileVisualizer.h>
#include "std_srvs/Empty.h"
#include "std_srvs/EmptyRequest.h"
#include "std_srvs/EmptyResponse.h"
#include "Visualizer.h"

/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv) {
    QApplication application(argc,argv);
    TileVisualizer visualizer(argc, argv);

    visualizer.initialize();
    visualizer.run();
    application.connect(&application, SIGNAL(lastWindowClosed()), &application, SLOT(quit()));

    int result = application.exec();
    return result;

}
