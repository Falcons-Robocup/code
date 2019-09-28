 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * abstractHeightMapTest.cpp
 *
 *  Created on: Nov 16, 2017
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/abstractHeightMap.hpp"

/* SUT dependencies */


/* A simple heightmap for testing */
class dummyHeightMap : public abstractHeightMap
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
class simpleHeightMap : public abstractHeightMap
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

/* Testing the abstractHeightMap */
class abstractHeightMapTest : public TeamplayTest
{
public:
    abstractHeightMapTest()
    {
    }
    dummyHeightMap _dummyHeightMap;
    simpleHeightMap _simpleHeightMap;
};

TEST_F(abstractHeightMapTest, getOptimum)
{
    _dummyHeightMap.precalculate();
    auto dummy_optimum = _dummyHeightMap.getOptimum();

    EXPECT_EQ(50.0, _dummyHeightMap.getValueAtCoordinate(dummy_optimum));
    EXPECT_EQ(-4.0, dummy_optimum.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, dummy_optimum.y);  // 5 --> -7.75

    _simpleHeightMap.precalculate();
    auto simple_optimum = _simpleHeightMap.getOptimum();

    EXPECT_EQ(50.0, _simpleHeightMap.getValueAtCoordinate(simple_optimum));
    EXPECT_EQ(-5.25, simple_optimum.x);  // 3 --> -5.25
    EXPECT_EQ(-8.25, simple_optimum.y);  // 3 --> -8.25
}

TEST_F(abstractHeightMapTest, sumOfTwoIdenticalHeightmaps)
{
    _dummyHeightMap.precalculate();

    abstractHeightMap sum = _dummyHeightMap + _dummyHeightMap;
    auto optimum = sum.getOptimum();
    EXPECT_EQ(-4.0, optimum.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, optimum.y);  // 5 --> -7.75

    EXPECT_NEAR(sum.getValueAtCoordinate(sum.getOptimum()), 100.0, 0.1);
}

TEST_F(abstractHeightMapTest, sumOfTwoDifferentHeightmaps)
{
    _dummyHeightMap.precalculate();
    _simpleHeightMap.precalculate();

    abstractHeightMap sum = _dummyHeightMap + _simpleHeightMap;  //Testing initialization
    auto optimum = sum.getOptimum();
    EXPECT_EQ(5.75, optimum.x);   //max --> 5.75
    EXPECT_EQ(8.75, optimum.y);   //max --> 8.75

    EXPECT_NEAR(sum.getValueAtCoordinate(sum.getOptimum()), 100.0, 2.0);
}

TEST_F(abstractHeightMapTest, sumOfTwoDifferentHeightmapsWithFactors)
{
    _dummyHeightMap.precalculate();
    _simpleHeightMap.precalculate();

    abstractHeightMap sum;
    sum = _dummyHeightMap.scale(0.99) + _simpleHeightMap.scale(0.01);  //Testing assignment
    auto optimum = sum.getOptimum();
    EXPECT_EQ(-4.0, optimum.x);   // 8 --> -4.0
    EXPECT_EQ(-7.75, optimum.y);  // 5 --> -7.75

    EXPECT_NEAR(sum.getValueAtCoordinate(sum.getOptimum()), 49.5, 0.1);
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
