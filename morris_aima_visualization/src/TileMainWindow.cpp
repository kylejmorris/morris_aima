#include <VisualEntityFactory.h>
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
    qRegisterMetaType<morris_aima_msgs::TileEnvironmentInfo>("morris_aima_msgs::TileEnvironmentInfo");
    this->performanceMeasureText = new QGraphicsTextItem(QString::fromStdString("Performance Measure: 0"));
    performanceMeasureText->setDefaultTextColor(Qt::blue);
    performanceMeasureText->setPos(FRAME_WIDTH*0.3,FRAME_HEIGHT*0.82);
    this->scene->addItem(performanceMeasureText);
    this->view->setViewportUpdateMode(QGraphicsView::NoViewportUpdate);
    view->setScene(scene);
    setCentralWidget(this->view);
    this->show();
    return true;
}

void TileMainWindow::closeWindow() {

}

void TileMainWindow::update(const morris_aima_msgs::TileEnvironmentInfo &msg) {
    if(updatingEnabled) {
        int performanceMeasureInt = msg.performance_measure;
        std::stringstream out;
        out << "Performance Measure: " << performanceMeasureInt;
        performanceMeasureText->setPlainText(QString::fromStdString(out.str()));
        performanceMeasureText->setZValue(4);

        this->grid->clean();
        //generating entities and their locations on the grid
        for(int currEntity = 0; currEntity<msg.entities.size(); currEntity++) {
            morris_aima_msgs::TileEntityInfo entity = msg.entities.at(currEntity);
                std::string type = entity.type;
                VisualEntity *currentEntity = VisualEntityFactory::createEntity(type, (int) (grid->getTileSize()), "");
                int x = entity.location_x;
                int y = entity.location_y;
                this->grid->addEntity(currentEntity, x, y);
        }
        this->view->viewport()->update();
    } else {
    }
}

void TileMainWindow::enableUpdating() {
    this->updatingEnabled = true;
}

void TileMainWindow::setParameters(int width, int height) {
    this->grid = new VisualTileGrid(width, height, this->view->width()*0.8,this->view->height()*0.8);
    grid->moveBy(view->width()*0.2,view->height()*0.2);
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

