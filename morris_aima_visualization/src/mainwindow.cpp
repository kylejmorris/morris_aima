/*
 * Main visualizer window.
 * @author: Kyle Morris
 * Purpose: This file couples the QT based Visualizer and ros framework vizualizer to provide a gui that updates upon receiving ROS signals.
 */

#include <QApplication>
#include <QNodeRosVisualizer.h>
#include <QThread>
#include <QTimer>
#include <sys/socket.h>

int main(int argc, char **argv) {
    QApplication application(argc,argv);
    QThread *thread = new QThread;
    QNodeRosVisualizer *vizNode = new QNodeRosVisualizer;
    QTimer *timer = new QTimer;

    vizNodemoveToThread(thread);
    vizNode.initialize(argc, argv);
    thread->start();

    while(true) {
        application.processEvents();
    }
}

