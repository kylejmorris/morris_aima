/* Class: QNodeRosVisualizer
 * Author: Kyle Morris
 * Date: July 24, 2016
 * Description: This handles creation of a ros node used while QT is being run.
 * A visualizer just uses QT, however if ros is being used, we must separate the ros thread from
 * QApplication thread. This wrapper is the solution.
 */
#ifndef MORRIS_AIMA_VISUALIZATION_QNODEROSVISUALIZER_H
#define MORRIS_AIMA_VISUALIZATION_QNODEROSVISUALIZER_H
#include "Visualizer.h"
#include <QObject>
class QObject;
class QNodeRosVisualizer :public  QObject{
    Q_OBJECT
    /**
     * The visualizer we're dealing with;
     */
    Visualizer *visualizer;
public:

    bool isValidWorldType(std::string worldType);

    /**
     * Set up the ros node and hooks.
     */
    void initialize(int argc, char **argv);


public slots:
    /**
     * Run ros main loop
     */
    void run();
    void render();

};


#endif //MORRIS_AIMA_VISUALIZATION_QNODEROSVISUALIZER_H
