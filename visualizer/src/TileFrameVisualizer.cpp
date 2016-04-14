#include <VisualTile.h>
#include "TileFrameVisualizer.h"
#include "FrameVisualizer.h"

void TileFrameVisualizer::update(std::string state) {
}

void TileFrameVisualizer::render() {
}

TileFrameVisualizer::TileFrameVisualizer(int width, int height, std::string name) {
  FrameVisualizer::Construct(width, height, name);
  Construct(width, height, name);
}

void TileFrameVisualizer::Construct(int width, int height, std::string name) {
  this->frameWidth = width;
  this->frameHeight = height;
  this->frameName = name;
  this->grid = new VisualTileGrid(width/2, height/2);
  this->getScene()->addItem(this->grid);
}

TileFrameVisualizer::TileFrameVisualizer() {
  Construct(this->frameWidth,this->frameHeight, "Tile Frame Visualizer");
}

TileFrameVisualizer::~TileFrameVisualizer() {
  delete this->grid;
}
