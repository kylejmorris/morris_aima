#include "TileMainWindow.h"
#include "VisualTileGrid.h"
TileMainWindow::TileMainWindow(int argc, char **argv, QWidget *parent) : QMainWindow(parent) {
    this->updatingEnabled = false;
    resize(this->FRAME_WIDTH, this->FRAME_HEIGHT);
    setWindowTitle(this->frameTitle);
}

bool TileMainWindow::initialize() {
    view = new QGraphicsView(this);
    scene = new QGraphicsScene(0,0,FRAME_WIDTH*0.9,FRAME_HEIGHT*0.9);
    this->performanceMeasure = new QGraphicsTextItem(QString::fromStdString("Performance Measure: 0"));
    performanceMeasure->setDefaultTextColor(Qt::blue);
    performanceMeasure->moveBy(0, 25);
    this->scene->addItem(performanceMeasure);
    this->view->setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
    view->setScene(scene);
    setCentralWidget(this->view);
    this->show();
    return true;
}

void TileMainWindow::closeWindow() {

}

void TileMainWindow::update(morris_aima_msgs::TileEnvironmentInfo &msg) {
    this->view->viewport()->update();
}

void TileMainWindow::enableUpdating() {
    this->updatingEnabled = true;
}

void TileMainWindow::setParameters(int width, int height, QString name) {
    VisualTileGrid *grid = new VisualTileGrid(width, height, 500,500);

    this->scene->addItem(grid);
    //this->grid(height, width, view->width()*0.9, view->height()*0.9);
    //this->scene->addItem(grid);
    this->view->viewport()->update();
}

void TileMainWindow::disableUpdating() {
    this->updatingEnabled = false;
}

void TileMainWindow::resetWindow() {

}

