 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * obstacleStoreTest.cpp
 *
 *  Created on: Jan 5, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/stores/obstacleStore.hpp"

/* SUT dependencies */



class obstacleStoreTest : public TeamplayTest
{
public:
    obstacleStoreTest()
    {
        obstacleStore::getInstance().clear();

        obstacle obs1(Position2D(3.5, -2.5, 0.0));
        obstacle obs2(Position2D(-1.2, -2.3, -3.4), Velocity2D(1.2, 2.3, 3.4));

        obstacleStore::getInstance().addObstacle(obs1);
        obstacleStore::getInstance().addObstacle(obs2);
    }
};

TEST_F(obstacleStoreTest, getAllObstacles)
{
    auto obstacles = obstacleStore::getInstance().getAllObstacles();
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(1).getLocation());
}

TEST_F(obstacleStoreTest, getAllObstaclesSortedByDistanceTo)
{
    auto obstacles = obstacleStore::getInstance().getAllObstaclesSortedByDistanceTo(Point2D(-1.5, -2.5));
    EXPECT_EQ(2, obstacles.size());
    EXPECT_EQ(Point2D(-1.2, -2.3), obstacles.at(0).getLocation());
    EXPECT_EQ(Point2D( 3.5, -2.5), obstacles.at(1).getLocation());
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
