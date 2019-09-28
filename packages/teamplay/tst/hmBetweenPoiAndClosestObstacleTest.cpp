 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * hmBetweenPoiAndClosestObstacleTest.cpp
 *
 *  Created on: Jun 15, 2018
 *      Author: Coen Tempelaars
 */

/* Include testframework */
#include "teamplayTest.hpp"

/* SUT */
#include "int/heightmaps/hmBetweenPoiAndClosestObstacle.hpp"

/* SUT dependencies */
#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"


/* Testing the 'between POI and closest obstacle' heightmap */

class hmBetweenPoiAndClosestObstacleTest : public TeamplayTest
{
public:
    hmBetweenPoiAndClosestObstacleTest()
    {
        ballStore::getBall().reset();
        ballStore::getBall().setPosition(Point3D(1.0, 1.0, 0.0));

        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D( 3.0, -1.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  1.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.0,  1.2, 0.0)));

        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot());
    }
    hmBetweenPoiAndClosestObstacle _hmBetweenPoiAndClosestObstacle;
    parameterMap_t _parameters;
};

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiBall)
{
    _parameters["POI"] = "ball";

    _hmBetweenPoiAndClosestObstacle.refine(_parameters);
    _hmBetweenPoiAndClosestObstacle.generateJPG("tst_hmBetweenPoiAndClosestObstacle_ball");
}

TEST_F(hmBetweenPoiAndClosestObstacleTest, poiOwnGoal)
{
    _parameters["POI"] = "P_OWN_GOALLINE_CENTER";

    _hmBetweenPoiAndClosestObstacle.refine(_parameters);
    _hmBetweenPoiAndClosestObstacle.generateJPG("tst_hmBetweenPoiAndClosestObstacle_ownGoal");
}


int main(int argc, char **argv)
{
    InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
