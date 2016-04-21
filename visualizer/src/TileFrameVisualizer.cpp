#include <VisualTile.h>
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
  this->grid->addEntity(VisualEntityFactory::createEntity("vacuum", state), new TileLocation(1,0));
}

TileFrameVisualizer::~TileFrameVisualizer() {
  delete this->grid;
}
