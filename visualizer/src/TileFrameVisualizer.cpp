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
  this->setFrameName(name);
  this->grid = new VisualTileGrid(rows, cols, this->getFrameWidth(), this->getFrameHeight());
  this->getScene()->addItem(this->grid);
  this->updateFrameTitle();
}

void TileFrameVisualizer::update(std::string state) {
  Json::Value root;
  Json::Reader reader;
  bool success = reader.parse(state,root,false);

  if(success) {
    this->grid->clean();
    //generating entities and their locations on the grid
    for(auto entity : root["Environment"]["entities"]) {
      VisualEntity *currentEntity = VisualEntityFactory::createEntity(entity["type"].asString(), "");
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
