#ifndef TILEENVIRONMENTSTATE_TESTS
#define TILEENVIRONMENTSTATE_TESTS

#include <Agent.h>
#include "gtest/gtest.h"
#include "Entity.h"
#include "TileEnvironmentState.h"
#include "TileLocation.h"

class TileEnvironmentState_Tests : public ::testing::Test {
public:
    Entity *e = new Agent;
    TileEnvironmentState *state = new TileEnvironmentState(4,4);

    virtual void setup() {
        e = new Agent;
    }

    virtual void TearDown() {
        delete e;
        delete state;
    }
};

TEST_F(TileEnvironmentState_Tests, moveEntity_successful_move) {
    TileLocation *originalLocation = new TileLocation(0,0);
    TileLocation *targetLocation = new TileLocation(1,1);
    int entityId = e->getId();

    state->add(e,originalLocation);
    bool result = state->moveEntity(entityId, targetLocation);
    EXPECT_EQ(result, true);
    TileLocation *found = state->getLocationOf(entityId);
    EXPECT_EQ(found->getX(),1);
    EXPECT_EQ(found->getY(),1);
    delete found;
    delete targetLocation;
    delete originalLocation;
}

TEST_F(TileEnvironmentState_Tests, moveEntity_failed_move_out_of_bounds) {
    TileLocation *originalLocation = new TileLocation(0,0);
    TileLocation *targetLocation = new TileLocation(4,8);

    int entityId = e->getId();
    state->add(e,originalLocation);
    bool result = state->moveEntity(entityId, targetLocation);
    EXPECT_EQ(result, false);
    TileLocation *found = state->getLocationOf(entityId);
    EXPECT_EQ(found->getX(),0);
    EXPECT_EQ(found->getY(),0);

    delete found;
    delete targetLocation;
    delete originalLocation;
}
#endif