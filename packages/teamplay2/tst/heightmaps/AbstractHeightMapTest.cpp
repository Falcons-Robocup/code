// Copyright 2021-2022 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * abstractHeightMapTest.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "../TeamplayTest.hpp"

/* SUT */
#include "int/heightmaps/AbstractHeightMap.hpp"

/* SUT dependencies */
#include "cEnvironmentField.hpp"


/* A simple heightmap for testing */
class DummyHeightMap : public AbstractHeightMap
{
public:
    void precalculate()
    {
        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                _heightMap(i, j).setValue((40.0 * i) / getNrOfHeightMapFieldsInX()
                                        + (10.0 * j) / getNrOfHeightMapFieldsInY());
            }
        }
        _heightMap(8, 5).setValue(50.0);
    }
};

/* Another simple heightmap for testing */
class SimpleHeightMap : public AbstractHeightMap
{
public:
    void precalculate()
    {
        for (unsigned int i = 0; i < getNrOfHeightMapFieldsInX(); i++)
        {
            for (unsigned int j = 0; j < getNrOfHeightMapFieldsInY(); j++)
            {
                _heightMap(i, j).setValue((10.0 * i) / getNrOfHeightMapFieldsInX()
                                        + (40.0 * j) / getNrOfHeightMapFieldsInY());
            }
        }
        _heightMap(3, 3).setValue(50.0);
    }
};

/* Testing the AbstractHeightMap */
class AbstractHeightMapTest : public TeamplayTest
{
public:
    AbstractHeightMapTest()
    {
    }
    DummyHeightMap _dummyHeightMap;
    SimpleHeightMap _simpleHeightMap;
};

TEST_F(AbstractHeightMapTest, getOptimum)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    _dummyHeightMap.precalculate();
    auto dummy_optimum = _dummyHeightMap.getOptimum();

    EXPECT_EQ(50.0, _dummyHeightMap.getFieldAtCoordinate(dummy_optimum._center).getValue());
    EXPECT_EQ(-4.0, dummy_optimum._center.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, dummy_optimum._center.y);  // 5 --> -7.75

    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");
    _simpleHeightMap.precalculate();
    auto simple_optimum = _simpleHeightMap.getOptimum();

    EXPECT_EQ(50.0, _simpleHeightMap.getFieldAtCoordinate(simple_optimum._center).getValue());
    EXPECT_EQ(-5.25, simple_optimum._center.x);  // 3 --> -5.25
    EXPECT_EQ(-8.25, simple_optimum._center.y);  // 3 --> -8.25
}

TEST_F(AbstractHeightMapTest, sumOfTwoIdenticalHeightmaps)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    _dummyHeightMap.precalculate();

    AbstractHeightMap sum = _dummyHeightMap + _dummyHeightMap;
    auto optimum = sum.getOptimum();
    EXPECT_EQ(-4.0, optimum._center.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, optimum._center.y);  // 5 --> -7.75

    EXPECT_NEAR(sum.getFieldAtCoordinate(optimum._center).getValue(), 100.0, 0.1);
}

TEST_F(AbstractHeightMapTest, sumOfTwoDifferentHeightmaps)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    _dummyHeightMap.precalculate();
    _simpleHeightMap.precalculate();

    AbstractHeightMap sum = _dummyHeightMap + _simpleHeightMap;  //Testing initialization
    auto optimum = sum.getOptimum();
    EXPECT_EQ(5.75, optimum._center.x);   //max --> 5.75
    EXPECT_EQ(8.75, optimum._center.y);   //max --> 8.75

    EXPECT_NEAR(sum.getFieldAtCoordinate(optimum._center).getValue(), 100.0, 2.0);
}

TEST_F(AbstractHeightMapTest, sumOfTwoDifferentHeightmapsWithFactors)
{
    cEnvironmentField::getInstance().loadConfig("cEnvironment12x18");

    _dummyHeightMap.precalculate();
    _simpleHeightMap.precalculate();

    AbstractHeightMap sum;
    sum = _dummyHeightMap.scale(0.99) + _simpleHeightMap.scale(0.01);  //Testing assignment
    auto optimum = sum.getOptimum();
    EXPECT_EQ(-4.0, optimum._center.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, optimum._center.y);  // 5 --> -7.75

    EXPECT_NEAR(sum.getFieldAtCoordinate(optimum._center).getValue(), 49.5, 0.1);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
