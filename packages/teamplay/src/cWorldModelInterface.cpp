 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelInterface.cpp
 *
 *  Created on: Sep 15, 2015
 *      Author: Ivo Matthijssen
 */
#include <stdexcept>

#include "int/rules/ruleStimulatePassing.hpp"
#include "int/stores/ballStore.hpp"
#include "int/stores/obstacleStore.hpp"
#include "int/stores/robotStore.hpp"
#include "int/cWorldModelInterface.hpp"

#include "cDiagnostics.hpp"

using namespace teamplay;

cWorldModelInterface::cWorldModelInterface()
{

}

cWorldModelInterface::~cWorldModelInterface()
{

}

// getters are used all over teamplay (and should be phased out)
// setters are used by teamplay tests (and should be phased out)
// store is called by cRosAdapterWorldModel

void cWorldModelInterface::getBallPossession(ballPossession_struct_t &ballPossession)
{
    ballPossession = _ballPossession;
}

void cWorldModelInterface::setBallPossession(ballPossession_struct_t const &ballPossession)
{
    _ballPossession = ballPossession;

    if (ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER)
    {
        ruleStimulatePassing::getInstance().robotClaimsBall(ballPossession.robotID);
    }
}

void cWorldModelInterface::store (const teamplay::worldModelInfo& wmInfo)
{
    try
    {
        /* Clear the stores */
        teamplay::ballStore::getBall().reset();
        teamplay::obstacleStore::getInstance().clear();
        teamplay::robotStore::getInstance().clear();

        /* Store the own robot */
        auto own_robot = robot(wmInfo.ownRobot.number, treeEnum::R_ROBOT_STOP, wmInfo.ownRobot.position, wmInfo.ownRobot.velocity);

        if ((wmInfo.ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER) &&
                (wmInfo.ballPossession.robotID == wmInfo.ownRobot.number))
        {
            own_robot.claimsBallPossession();
        }

        teamplay::robotStore::getInstance().addOwnRobot(own_robot);

        /* Store the active teammembers */
        for (auto member = wmInfo.activeTeammembers.begin(); member != wmInfo.activeTeammembers.end(); member++)
        {
            const Position2D pos(member->second.position.x, member->second.position.y, member->second.position.getPhi());
            const Velocity2D vel(member->second.velocity.x, member->second.velocity.y, member->second.velocity.getPhi());
            robot teammate(member->first, treeEnum::R_ROBOT_STOP, pos, vel);

            if (wmInfo.ballPossession.robotID == member->first)
            {
                teammate.claimsBallPossession();
            }

            teamplay::robotStore::getInstance().addTeammate(teammate);
        }

        /* Store the obstacles */
        for (auto obstacle = wmInfo.obstacles.begin(); obstacle != wmInfo.obstacles.end(); obstacle++)
        {
            const Position2D pos(obstacle->second.position.x, obstacle->second.position.y, obstacle->second.position.getPhi());
            const Velocity2D vel(obstacle->second.velocity.x, obstacle->second.velocity.y, obstacle->second.velocity.getPhi());

            teamplay::obstacleStore::getInstance().addObstacle(teamplay::obstacle(pos, vel));
        }

        /* Store the ball */
        if (wmInfo.ball)
        {
            teamplay::ballStore::getBall().setPosition(wmInfo.ball->position);
            teamplay::ballStore::getBall().setVelocity(wmInfo.ball->velocity);
        }
        else
        {
            teamplay::ballStore::getBall().setPositionUnknown();
            teamplay::ballStore::getBall().setVelocityUnknown();
        }

        /* Store the ball possession information */
        _ballPossession = wmInfo.ballPossession;

        if (wmInfo.ballPossession.possessionType == ballPossessionEnum::TEAMMEMBER)
        {
            teamplay::ballStore::getBall().setPositionClaimed(Point3D(wmInfo.ballPossession.ballClaimedLocation.x, wmInfo.ballPossession.ballClaimedLocation.y, 0.0));
            ruleStimulatePassing::getInstance().robotClaimsBall(wmInfo.ballPossession.robotID);
        }
        else
        {
            teamplay::ballStore::getBall().setPositionClaimedUnknown();
        }
    }
    catch (std::exception& e)
    {
        TRACE_ERROR("Caught exception while storing worldmodel data: %s", e.what());
        throw std::runtime_error(std::string("Caught exception while storing worldmodel data: ") + e.what());
    }
}

