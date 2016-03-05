#include "TileEnvironmentState.h"

bool TileEnvironmentState::isValid() {
    return false;
}

int TileEnvironmentState::compareTo(EnvironmentState *other) {
    return 0;
}

TileEnvironmentState::TileEnvironmentState(int width, int height) : WIDTH(width), HEIGHT(height) {

}
