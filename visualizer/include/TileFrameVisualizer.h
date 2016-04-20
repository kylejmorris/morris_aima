/**
* CLASS: TileFrameVisualizer
* DATE: 31/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: Handles QT support for a TileEnvironment scene to be displayed.
* This class will allow for a grid to be generated, and various objects to be displayed on screen.
*/

#ifndef MORRIS_AIMA_TILEFRAMEVISUALIZER_H
#define MORRIS_AIMA_TILEFRAMEVISUALIZER_H
#include "FrameVisualizer.h"
#include "TileGrid.h"
#include "VisualTileGrid.h"

class TileFrameVisualizer : public FrameVisualizer {
private:
    VisualTileGrid *grid;
    VisualTile *testTile;

public:
    TileFrameVisualizer(int width, int height, std::string name);
    TileFrameVisualizer();
    virtual ~TileFrameVisualizer();


    virtual void Construct(int width, int height, std::string name) override;

    virtual void update(std::string state) override;
};


#endif //MORRIS_AIMA_TILEFRAMEVISUALIZER_H
