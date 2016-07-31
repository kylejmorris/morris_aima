#include "TileMainWindow.h"
TileMainWindow::TileMainWindow(int argc, char **argv) {
    this->updatingEnabled = false;
}

bool TileMainWindow::initialize() {
    return false;
}

void TileMainWindow::closeWindow() {

}

void TileMainWindow::update(morris_aima_msgs::TileEnvironmentInfo &msg) {

}

void TileMainWindow::enableUpdating() {
    this->updatingEnabled = true;
}

void TileMainWindow::setParameters(int width, int height, std::string name) {

}

void TileMainWindow::disableUpdating() {
    this->updatingEnabled = false;
}

void TileMainWindow::resetWindow() {

}

