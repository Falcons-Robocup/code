 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * standaloneTuningTool.cpp
 *
 *  Created on: Dec 20, 2017
 *      Author: Coen Tempelaars
 */

#include <iostream>

#include "ros/ros.h"

#include "int/adapters/configuration/cConfigHeightmaps.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/heightMapStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;


int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "standaloneTuningTool");
        ros::Time::init();

        ros::Duration duration(0.33);

        cConfigHeightmaps::getInstance();
        configurationStore::getConfiguration().setHeightMapsGeneratePictures(true);

        ballStore::getBall().reset();
        ballStore::getBall().setPosition(Point3D(-4.25, 5.0, 0.0));

        robotStore::getInstance().clear();
        robotStore::getInstance().addOwnRobot(robot(1, treeEnum::ATTACKER_ASSIST, Position2D(-3.0, 0.0, 0.0), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(2, treeEnum::DEFENDER_MAIN, Position2D(-2.0, -5.0, 0.0), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(3, treeEnum::ATTACKER_MAIN, Position2D(3.0, 3.0, 0.0), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(4, treeEnum::DEFENDER_ASSIST, Position2D(2.0, -3.0, 0.0), Velocity2D()));
        robotStore::getInstance().addTeammate(robot(5, treeEnum::R_GOALKEEPER, Position2D(0.0, -7.5, 0.0), Velocity2D()));

        obstacleStore::getInstance().clear();
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.75, 6.5, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(-2.75, 3.0, 0.0)));
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(0.0, -3.5, 0.0)));

        while (ros::ok())
        {
            parameterMap_t params;
            teamplay::heightMapStore::getInstance().precalculateAll();

            /* Define default values of all parameters */
            params["onSide"] = "both";
            params["POI"] = "ball";

            /* Generate a JPG for every action */
            for (auto& kv : actionMapping)
            {
                teamplay::heightMapStore::getInstance().generateJPG(kv.second, kv.first, params);
            }

            /* Generate JPGs for specific actions with specific parameters */
            params["onSide"] = "withBall";
            teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::POSITION_FOR_OWN_SETPIECE, "positionForOwnSetpieceOnSideWithBall", params);

            params["onSide"] = "withoutBall";
            teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::POSITION_FOR_OWN_SETPIECE, "positionForOwnSetpieceOnSideWithoutBall", params);

            params["POI"] = "P_OWN_GOALLINE_CENTER";
            teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::DEFEND_ATTACKING_OPPONENT, "defendAttackingOpponentFromOwnGoal", params);

            ros::spinOnce();
            duration.sleep();
        }
    }
    catch (std::exception &e)
    {
        std::cerr << "Caught exception: " << e.what();
        throw;
    }
    catch (...)
    {
        std::cerr << "Caught unknown exception.";
        throw;
    }
}
