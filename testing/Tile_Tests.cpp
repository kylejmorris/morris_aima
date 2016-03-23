#ifndef TILE_TEST_H
#define TILE_TEST_H

#include "gtest/gtest.h"
#include "Entity.h"
#include "Agent.h"
#include "Tile.h"
#include "json/json.h"

class Tile_Tests : public ::testing::Test {
protected:

    virtual void SetUp() {
        a->addEntity(ag);
    }

    virtual void TearDown() {
        delete ag;
        delete a;
    }

    Agent *ag = new Agent;
    Tile *a = new Tile;
};

//remove entity that does exist
TEST_F(Tile_Tests, removeEntity_That_Does_Exist_In_Tile) {
    int id = ag->getId();
    Entity *result = a->removeEntity(id);
    EXPECT_EQ(result->getId(),id); //since entity with 0 exists, this should be true
}

TEST_F(Tile_Tests, removeEntity_That_Does_Not_Exist_In_Tile) {
    int id = ag->getId();
    ASSERT_TRUE(a->removeEntity(id+1)==NULL);
}

TEST_F(Tile_Tests, exists_Entity_That_Does_Exist) {
    int id = ag->getId();
    ASSERT_TRUE(a->exists(id));
}

TEST_F(Tile_Tests, exists_Entity_That_Does_Not_Exist) {
    ASSERT_FALSE(a->exists(666));
}
#endif