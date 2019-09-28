 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cConfigHeightmaps.cpp
 *
 *  Created on: Dec 5, 2017
 *      Author: Coen Tempelaars
 */

#include "int/adapters/configuration/cConfigHeightmaps.hpp"

#include <ros/param.h>
#include <pwd.h> // for getpwuid()

#include "FalconsCommon.h"
#include "tracing.hpp"

#include "int/stores/configurationStore.hpp"

cConfigHeightmaps::cConfigHeightmaps()
{
    loadConfigYaml();
}

cConfigHeightmaps::~cConfigHeightmaps()
{

}

void cConfigHeightmaps::loadConfigYaml()
{
    teamplay::teamplayHeightmapsConfig config;

    /* Execute loading of ros params */
    struct passwd *pw = getpwuid(getuid());
    std::string configFileCmd("rosparam load ");
    configFileCmd.append(pw->pw_dir);
    configFileCmd.append("/falcons/code/config/teamplayHeightmaps.yaml");
    int dummy_val = system(configFileCmd.c_str());
    TRACE("heightmap parameters loaded, returncode=%d", dummy_val);
    dummy_val = 0;

    /* Bind the reconfiguration function */
    _srv.reset(new dynamic_reconfigure::Server<teamplay::teamplayHeightmapsConfig>(ros::NodeHandle("~/heightmaps")));
    _f = boost::bind(&cConfigHeightmaps::reconfig_cb, this, _1, _2);
    _srv->setCallback(_f);

    /* Get standard configuration and update values */
    while(!ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmAvoidBall",
            config.actionMoveToFreeSpot_hmAvoidBall))
    {
        std::cout << "Teamplay heightmap parameters not loaded yet??" << std::endl;
        sleep(5);
    }

    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmAvoidBall",
            config.actionDefendAttackingOpponent_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmAvoidObstacles",
            config.actionDefendAttackingOpponent_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmAvoidTeamMates",
            config.actionDefendAttackingOpponent_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmBetweenPoiAndClosestObstacle",
            config.actionDefendAttackingOpponent_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmCloseToBallClaimedLocation",
            config.actionDefendAttackingOpponent_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmCloseToOwnPos",
            config.actionDefendAttackingOpponent_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmInFrontOfOppGoal",
            config.actionDefendAttackingOpponent_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmNearObstacles",
            config.actionDefendAttackingOpponent_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmNearOwnGoal",
            config.actionDefendAttackingOpponent_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmObstaclesBlockingBall",
            config.actionDefendAttackingOpponent_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmObstaclesBlockingOppGoal",
            config.actionDefendAttackingOpponent_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDefendAttackingOpponent_hmObstaclesBlockingTeammates",
            config.actionDefendAttackingOpponent_hmObstaclesBlockingTeammates);

    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmAvoidBall",
            config.actionDribbleForPass_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmAvoidObstacles",
            config.actionDribbleForPass_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmAvoidTeamMates",
            config.actionDribbleForPass_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmBetweenPoiAndClosestObstacle",
            config.actionDribbleForPass_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmCloseToBallClaimedLocation",
            config.actionDribbleForPass_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmCloseToOwnPos",
            config.actionDribbleForPass_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmInFrontOfOppGoal",
            config.actionDribbleForPass_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmNearObstacles",
            config.actionDribbleForPass_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmNearOwnGoal",
            config.actionDribbleForPass_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmObstaclesBlockingBall",
            config.actionDribbleForPass_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmObstaclesBlockingOppGoal",
            config.actionDribbleForPass_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForPass_hmObstaclesBlockingTeammates",
            config.actionDribbleForPass_hmObstaclesBlockingTeammates);

    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmAvoidBall",
            config.actionDribbleForShot_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmAvoidObstacles",
            config.actionDribbleForShot_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmAvoidTeamMates",
            config.actionDribbleForShot_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmBetweenPoiAndClosestObstacle",
            config.actionDribbleForShot_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmCloseToBallClaimedLocation",
            config.actionDribbleForShot_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmCloseToOwnPos",
            config.actionDribbleForShot_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmInFrontOfOppGoal",
            config.actionDribbleForShot_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmNearObstacles",
            config.actionDribbleForShot_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmNearOwnGoal",
            config.actionDribbleForShot_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmObstaclesBlockingBall",
            config.actionDribbleForShot_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmObstaclesBlockingOppGoal",
            config.actionDribbleForShot_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionDribbleForShot_hmObstaclesBlockingTeammates",
            config.actionDribbleForShot_hmObstaclesBlockingTeammates);

    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmAvoidBall",
            config.actionMoveToFreeSpot_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmAvoidObstacles",
            config.actionMoveToFreeSpot_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmAvoidTeamMates",
            config.actionMoveToFreeSpot_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmBetweenPoiAndClosestObstacle",
            config.actionMoveToFreeSpot_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmCloseToBallClaimedLocation",
            config.actionMoveToFreeSpot_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmCloseToOwnPos",
            config.actionMoveToFreeSpot_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmInFrontOfOppGoal",
            config.actionMoveToFreeSpot_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmNearObstacles",
            config.actionMoveToFreeSpot_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmNearOwnGoal",
            config.actionMoveToFreeSpot_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmObstaclesBlockingBall",
            config.actionMoveToFreeSpot_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmObstaclesBlockingOppGoal",
            config.actionMoveToFreeSpot_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionMoveToFreeSpot_hmObstaclesBlockingTeammates",
            config.actionMoveToFreeSpot_hmObstaclesBlockingTeammates);

    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmAvoidBall",
            config.actionPositionForOppSetpiece_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmAvoidObstacles",
            config.actionPositionForOppSetpiece_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmAvoidTeamMates",
            config.actionPositionForOppSetpiece_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmBetweenPoiAndClosestObstacle",
            config.actionPositionForOppSetpiece_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmCloseToBallClaimedLocation",
            config.actionPositionForOppSetpiece_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmCloseToOwnPos",
            config.actionPositionForOppSetpiece_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmInFrontOfOppGoal",
            config.actionPositionForOppSetpiece_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmNearObstacles",
            config.actionPositionForOppSetpiece_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmNearOwnGoal",
            config.actionPositionForOppSetpiece_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmObstaclesBlockingBall",
            config.actionPositionForOppSetpiece_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmObstaclesBlockingOppGoal",
            config.actionPositionForOppSetpiece_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOppSetpiece_hmObstaclesBlockingTeammates",
            config.actionPositionForOppSetpiece_hmObstaclesBlockingTeammates);

    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmAvoidBall",
            config.actionPositionForOwnSetpiece_hmAvoidBall);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmAvoidObstacles",
            config.actionPositionForOwnSetpiece_hmAvoidObstacles);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmAvoidTeamMates",
            config.actionPositionForOwnSetpiece_hmAvoidTeamMates);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmBetweenPoiAndClosestObstacle",
            config.actionPositionForOwnSetpiece_hmBetweenPoiAndClosestObstacle);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmCloseToBallClaimedLocation",
            config.actionPositionForOwnSetpiece_hmCloseToBallClaimedLocation);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmCloseToOwnPos",
            config.actionPositionForOwnSetpiece_hmCloseToOwnPos);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmInFrontOfOppGoal",
            config.actionPositionForOwnSetpiece_hmInFrontOfOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmNearObstacles",
            config.actionPositionForOwnSetpiece_hmNearObstacles);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmNearOwnGoal",
            config.actionPositionForOwnSetpiece_hmNearOwnGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmObstaclesBlockingBall",
            config.actionPositionForOwnSetpiece_hmObstaclesBlockingBall);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmObstaclesBlockingOppGoal",
            config.actionPositionForOwnSetpiece_hmObstaclesBlockingOppGoal);
    ros::param::get("teamplay_main/heightmaps/actionPositionForOwnSetpiece_hmObstaclesBlockingTeammates",
            config.actionPositionForOwnSetpiece_hmObstaclesBlockingTeammates);

    /* Call configuration file */
    reconfig_cb(config, 0);
}

void cConfigHeightmaps::reconfig_cb(teamplay::teamplayHeightmapsConfig &config, uint32_t level)
{
    // TODO: the list below will be a source of bugs because of typos. Should be refactored soon.
    using namespace teamplay;

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::AVOID_BALL,                       config.actionDefendAttackingOpponent_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::AVOID_OBSTACLES,                  config.actionDefendAttackingOpponent_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::AVOID_TEAM_MATES,                 config.actionDefendAttackingOpponent_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionDefendAttackingOpponent_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionDefendAttackingOpponent_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionDefendAttackingOpponent_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionDefendAttackingOpponent_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::NEAR_OBSTACLES,                   config.actionDefendAttackingOpponent_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::NEAR_OWN_GOAL,                    config.actionDefendAttackingOpponent_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::OBS_BLOCKING_BALL,                config.actionDefendAttackingOpponent_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionDefendAttackingOpponent_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DEFEND_ATTACKING_OPPONENT, heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionDefendAttackingOpponent_hmObstaclesBlockingTeammates);

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::AVOID_BALL,                       config.actionDribbleForPass_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::AVOID_OBSTACLES,                  config.actionDribbleForPass_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::AVOID_TEAM_MATES,                 config.actionDribbleForPass_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionDribbleForPass_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionDribbleForPass_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionDribbleForPass_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionDribbleForPass_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::NEAR_OBSTACLES,                   config.actionDribbleForPass_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::NEAR_OWN_GOAL,                    config.actionDribbleForPass_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::OBS_BLOCKING_BALL,                config.actionDribbleForPass_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionDribbleForPass_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_PASS,          heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionDribbleForPass_hmObstaclesBlockingTeammates);

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::AVOID_BALL,                       config.actionDribbleForShot_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::AVOID_OBSTACLES,                  config.actionDribbleForShot_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::AVOID_TEAM_MATES,                 config.actionDribbleForShot_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionDribbleForShot_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionDribbleForShot_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionDribbleForShot_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionDribbleForShot_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::NEAR_OBSTACLES,                   config.actionDribbleForShot_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::NEAR_OWN_GOAL,                    config.actionDribbleForShot_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::OBS_BLOCKING_BALL,                config.actionDribbleForShot_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionDribbleForShot_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::DRIBBLE_FOR_SHOT,          heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionDribbleForShot_hmObstaclesBlockingTeammates);

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::AVOID_BALL,                       config.actionMoveToFreeSpot_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::AVOID_OBSTACLES,                  config.actionMoveToFreeSpot_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::AVOID_TEAM_MATES,                 config.actionMoveToFreeSpot_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionMoveToFreeSpot_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionMoveToFreeSpot_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionMoveToFreeSpot_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionMoveToFreeSpot_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::NEAR_OBSTACLES,                   config.actionMoveToFreeSpot_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::NEAR_OWN_GOAL,                    config.actionMoveToFreeSpot_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::OBS_BLOCKING_BALL,                config.actionMoveToFreeSpot_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionMoveToFreeSpot_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::MOVE_TO_FREE_SPOT,         heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionMoveToFreeSpot_hmObstaclesBlockingTeammates);

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::AVOID_BALL,                       config.actionPositionForOppSetpiece_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::AVOID_OBSTACLES,                  config.actionPositionForOppSetpiece_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::AVOID_TEAM_MATES,                 config.actionPositionForOppSetpiece_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionPositionForOppSetpiece_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionPositionForOppSetpiece_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionPositionForOppSetpiece_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionPositionForOppSetpiece_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::NEAR_OBSTACLES,                   config.actionPositionForOppSetpiece_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::NEAR_OWN_GOAL,                    config.actionPositionForOppSetpiece_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::OBS_BLOCKING_BALL,                config.actionPositionForOppSetpiece_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionPositionForOppSetpiece_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OPP_SETPIECE, heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionPositionForOppSetpiece_hmObstaclesBlockingTeammates);

    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::AVOID_BALL,                       config.actionPositionForOwnSetpiece_hmAvoidBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::AVOID_OBSTACLES,                  config.actionPositionForOwnSetpiece_hmAvoidObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::AVOID_TEAM_MATES,                 config.actionPositionForOwnSetpiece_hmAvoidTeamMates);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::BETWEEN_POI_AND_CLOSEST_OBSTACLE, config.actionPositionForOwnSetpiece_hmBetweenPoiAndClosestObstacle);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::CLOSE_TO_BALL_CLAIMED_LOCATION,   config.actionPositionForOwnSetpiece_hmCloseToBallClaimedLocation);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::CLOSE_TO_OWN_POS,                 config.actionPositionForOwnSetpiece_hmCloseToOwnPos);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::IN_FRONT_OF_OPP_GOAL,             config.actionPositionForOwnSetpiece_hmInFrontOfOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::NEAR_OBSTACLES,                   config.actionPositionForOwnSetpiece_hmNearObstacles);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::NEAR_OWN_GOAL,                    config.actionPositionForOwnSetpiece_hmNearOwnGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::OBS_BLOCKING_BALL,                config.actionPositionForOwnSetpiece_hmObstaclesBlockingBall);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::OBS_BLOCKING_OPP_GOAL,            config.actionPositionForOwnSetpiece_hmObstaclesBlockingOppGoal);
    configurationStore::getConfiguration().setHeightMapFactorForAction(tpActionEnum::POSITION_FOR_OWN_SETPIECE, heightmapEnum::OBS_BLOCKING_TEAMMATES,           config.actionPositionForOwnSetpiece_hmObstaclesBlockingTeammates);
}
