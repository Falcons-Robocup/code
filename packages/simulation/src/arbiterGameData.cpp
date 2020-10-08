 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * arbiterGameData.cpp
 *
 *  Created on: Dec 29, 2018
 *      Author: Coen Tempelaars
 */

#include "int/arbiterGameData.hpp"

#include "cEnvironmentField.hpp"

const static float SCRUM_THRESHOLD__BALL_SPEED = 0.1;
const static float SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE = 1.0;

using boost::accumulators::rolling_mean;
using boost::accumulators::tag::rolling_window;


ArbiterGameData::ArbiterGameData()
: teamLastHoldingBall(TeamID::A)
, robotLastHoldingBall(RobotID::r1)
, ballSpeedAccumulator(rolling_window::window_size = 100)
, ballToRobotDistanceAccumulatorTeamA(rolling_window::window_size = 100)
, ballToRobotDistanceAccumulatorTeamB(rolling_window::window_size = 100)
{ }


void ArbiterGameData::refresh(const GameData& gameData)
{
    ball = gameData.ball;
    team = gameData.team;

    for (const auto& teampair: team)
    {
        for (const auto& robotpair: teampair.second)
        {
            const auto& robot = robotpair.second;
            if (robot.canKickBall(ball.getPosition()))
            {
                teamLastHoldingBall = teampair.first;
                robotLastHoldingBall = robotpair.first;
            }
        }
    }

    ballSpeedAccumulator(ball.getSpeed());

    if (team.find(TeamID::A) != team.end())
    {
        ballToRobotDistanceAccumulatorTeamA(getDistanceOfClosestRobotTo(ball.getLocation(), TeamID::A));
    }

    if (team.find(TeamID::B) != team.end())
    {
        ballToRobotDistanceAccumulatorTeamB(getDistanceOfClosestRobotTo(ball.getLocation(), TeamID::B));
    }
}


bool ArbiterGameData::ballIsInNegativeGoal() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y < -0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) < cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsInPositiveGoal() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y > 0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) < cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossNegativeGoalline() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y < -0.5 * cEnvironmentField::getInstance().getLength())
            &&  (abs(ballLocation.x) > cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossPositiveGoalline() const
{
    const auto ballLocation = ball.getLocation();

    return  (  (ballLocation.y > 0.5 * cEnvironmentField::getInstance().getLength())
            && (abs(ballLocation.x) > cEnvironmentField::getInstance().getGoalPostOffset())
            );
}

bool ArbiterGameData::ballIsAcrossSideLine() const
{
    return (abs(ball.getLocation().x) > (0.5 * cEnvironmentField::getInstance().getWidth()));
}

bool ArbiterGameData::isScrum() const
{
    return (  (rolling_mean(ballSpeedAccumulator) < SCRUM_THRESHOLD__BALL_SPEED)
           && (rolling_mean(ballToRobotDistanceAccumulatorTeamA) < SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE)
           && (rolling_mean(ballToRobotDistanceAccumulatorTeamB) < SCRUM_THRESHOLD__BALL_TO_ROBOT_DISTANCE)
    );
}
