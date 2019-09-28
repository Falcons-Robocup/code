 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * liveTuningTool.cpp
 *
 *  Created on: Apr 27, 2018
 *      Author: Coen Tempelaars
 */

#include <string>

#include "ros/ros.h"

#include "rosMsgs/t_worldmodel_team.h"

#include "int/adapters/configuration/cConfigHeightmaps.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/configurationStore.hpp"
#include "int/stores/heightMapStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/types/cActionTypes.hpp"
#include "int/utilities/trace.hpp"

using namespace teamplay;

void generate_heightmaps(const std::string& robot)
{
    parameterMap_t params;
    teamplay::heightMapStore::getInstance().precalculateAll();

    /* Define default values of all parameters */
    params["onSide"] = "both";
    params["POI"] = "ball";

    /* Generate a JPG for every action */
    for (auto& kv : actionMapping)
    {
        teamplay::heightMapStore::getInstance().generateJPG(kv.second, robot + kv.first, params);
    }

    /* Generate JPGs for specific actions with specific parameters */
    params["onSide"] = "withBall";
    teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::POSITION_FOR_OWN_SETPIECE, robot + "PositionForOwnSetpieceOnSideWithBall", params);

    params["onSide"] = "withoutBall";
    teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::POSITION_FOR_OWN_SETPIECE, robot + "PositionForOwnSetpieceOnSideWithoutBall", params);

    params["POI"] = "P_OWN_GOALLINE_CENTER";
    teamplay::heightMapStore::getInstance().generateJPG(tpActionEnum::DEFEND_ATTACKING_OPPONENT, robot + "DefendAttackingOpponentFromOwnGoal", params);
}

void cb_worldmodelUpdate (const rosMsgs::t_worldmodel_team::ConstPtr& msg)
{
    // Store the ball
    if (!msg->balls.empty())
    {
        auto ball = msg->balls.at(0);
        ballStore::getBall().setPosition(Point3D(ball.x, ball.y, ball.z));
    }
    else
    {
        ballStore::getBall().setPositionUnknown();
    }

    // Store the obstacles
    obstacleStore::getInstance().clear();
    for (auto it = msg->obstacles.begin(); it != msg->obstacles.end(); ++it)
    {
        obstacleStore::getInstance().addObstacle(obstacle(Position2D(it->x, it->y, it->phi)));
    }

    // For all robots: store them and generate heightmaps
    int own_robot_id = 1;
    for (auto outer_it = msg->robots.begin()+1; outer_it != msg->robots.end()-1; ++outer_it)
    {
        robotStore::getInstance().clear();
        int robot_id = 1;
        for (auto it = msg->robots.begin()+1; it != msg->robots.end()-1; ++it)
        {
            if (robot_id == own_robot_id)
            {
                robotStore::getInstance().addOwnRobot(robot(robot_id, treeEnum::R_ROBOT_STOP, Position2D(it->x, it->y, it->phi), Velocity2D(it->vx, it->vy, it->vphi)));
            }
            else
            {
                robotStore::getInstance().addTeammate(robot(robot_id, treeEnum::R_ROBOT_STOP, Position2D(it->x, it->y, it->phi), Velocity2D(it->vx, it->vy, it->vphi)));
            }
            robot_id++;
        }
        generate_heightmaps("r" + std::to_string(own_robot_id));
        own_robot_id++;
    }
}

int main(int argc, char **argv)
{
    try
    {
        ros::init(argc, argv, "liveTuningTool");
        ros::Time::init();

        ros::NodeHandle nh;
        ros::Subscriber subscription = nh.subscribe<rosMsgs::t_worldmodel_team>("/teamA/g_worldmodel_team", 1, &cb_worldmodelUpdate);

        cConfigHeightmaps::getInstance();
        configurationStore::getConfiguration().setHeightMapsGeneratePictures(true);

        ros::spin();
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
