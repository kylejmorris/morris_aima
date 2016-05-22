#include <VisualTile.h>
#include <json/json.h>
#include <iostream>
#include "TileFrameVisualizer.h"
#include "FrameVisualizer.h"
#include "VisualEntityFactory.h"

static int row = 0;

TileFrameVisualizer::TileFrameVisualizer() {
  Construct(this->frameWidth,this->frameHeight, "Tile Frame Visualizer");
}

TileFrameVisualizer::TileFrameVisualizer(int rows, int cols, std::string name) {
  Construct(rows, cols, name);
}

void TileFrameVisualizer::Construct(int rows, int cols, std::string name) {
  this->performanceMeasureText = new QGraphicsTextItem;
  this->performanceMeasureText->setDefaultTextColor(Qt::blue);
  this->performanceMeasureText->moveBy(0,this->getFrameHeight()/2);
  this->getScene()->addItem(performanceMeasureText);
  this->setFrameName(name);
  this->grid = new VisualTileGrid(rows, cols, this->getFrameWidth(), this->getFrameHeight());
  this->getScene()->addItem(this->grid);
  this->updateFrameTitle();
}

void TileFrameVisualizer::update(std::string state) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(state,root,false);
  int performanceMeasureInt = root["debug"]["performance_measure"].asInt();
  std::stringstream out;
  out << "Performance Measure: " << performanceMeasureInt;
  performanceMeasureText->setPlainText(QString::fromStdString(out.str()));
  performanceMeasureText->setZValue(4);

#ifndef NDEBUG
  std::cout << "TileFrameVisualizer.cpp: input state: " << state << "\n";
#endif
  if(success) {
    this->grid->clean();
    //generating entities and their locations on the grid
    for(auto entity : root["Environment"]["entities"]) {
      VisualEntity *currentEntity = VisualEntityFactory::createEntity(entity["type"].asString(), grid->getTileSize(), "");
      TileLocation currentLocation(entity["location"]["x"].asInt(), entity["location"]["y"].asInt());
      this->grid->addEntity(currentEntity, &currentLocation);
    }
  } else {
    std::cout << "Error updating TileFrameVisualizer: " << reader.getFormattedErrorMessages();
  }
}

TileFrameVisualizer::~TileFrameVisualizer() {
  delete this->grid;
}
