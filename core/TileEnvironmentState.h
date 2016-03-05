/**
* CLASS: TileEnvironmentState
* DATE: 03/03/16
* AUTHOR: Kyle Morris
* DESCRIPTION: A TileEnvironment will have a grid of tiles containing entities.
*/

#ifndef MORRIS_AIMA_TILEENVIRONMENTSTATE_H
#define MORRIS_AIMA_TILEENVIRONMENTSTATE_H
#include "EnvironmentState.h"

class TileEnvironmentState : public EnvironmentState {
private:
    const int WIDTH;
    const int HEIGHT;
public:
    TileEnvironmentState(int width, int height);
    virtual bool isValid() override;

    virtual int compareTo(EnvironmentState *other) override;
};

#endif //MORRIS_AIMA_TILEENVIRONMENTSTATE_H
