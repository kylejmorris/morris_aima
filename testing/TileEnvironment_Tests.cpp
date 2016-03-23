#ifndef TILE_ENVIRONMENT_STATE_TESTS
#define TILE_ENVIRONMENT_STATE_TESTS
#include "gtest/gtest.h"
#include "TileEnvironmentState.h"
#include "Agent.h"
#include "TileLocation.h"
#include "Location.h"
#include "TileEnvironment.h"

class TileEnvironment_Tests : public ::testing::Test {
protected:

    virtual void TearDown() {

    }
    TileEnvironment *environment = new TileEnvironment;
};

TEST_F(TileEnvironment_Tests, add_Several_Entities) {
    environment->add(new Agent(), new TileLocation(0,0));//adding to corner
    EXPECT_EQ(environment->getEntities().size(),1);
    environment->add(new Agent(), new TileLocation(4,4));
    EXPECT_EQ(environment->getEntities().size(),2);
    environment->add(new Agent(), new TileLocation(4,4)); //adding to same tile
    EXPECT_EQ(environment->getEntities().size(),3);
}
#endif