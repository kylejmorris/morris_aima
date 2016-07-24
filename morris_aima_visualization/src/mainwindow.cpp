//Main visualizer window

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
    vizNode->moveToThread(thread);
    vizNode->initialize(argc, argv);
    thread->start();
    while(true) {
        vizNode->run();
        application.processEvents();
    }
    //QObject::connect(thread, SIGNAL(started()), vizNode, SLOT(run())); //ros loop
    //QObject::connect(timer, SIGNAL(timeout()), vizNode, SLOT(run()));
}

