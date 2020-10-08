 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBOutputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include <sstream>

#include "../../include/int/adapters/MP_RTDBOutputAdapter.hpp"
#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

MP_RTDBOutputAdapter::MP_RTDBOutputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
}

MP_RTDBOutputAdapter::~MP_RTDBOutputAdapter()
{
}

void MP_RTDBOutputAdapter::setMotionSetpoint(actionTypeEnum action, Position2D pos, bool slow)
{
    std::ostringstream str;
    str << "action=" << static_cast<std::underlying_type<actionTypeEnum>::type>(action) << "; pos=" << pos.tostr() << "; slow=" << slow;
    TRACE_FUNCTION(str.str().c_str());

    T_MOTION_SETPOINT motionSetpoint;

    motionSetpoint.action = action;
    motionSetpoint.position.x = pos.x;
    motionSetpoint.position.y = pos.y;
    motionSetpoint.position.z = pos.phi;
    motionSetpoint.slow = slow;

    tprintf("             put MOTION_SETPOINT action=%s pos=[%6.2f, %6.2f, %6.2f] slow=%d", enum2str(motionSetpoint.action), motionSetpoint.position.x, motionSetpoint.position.y, motionSetpoint.position.z, motionSetpoint.slow);
    _rtdb->put(MOTION_SETPOINT, &motionSetpoint);
}

void MP_RTDBOutputAdapter::setShootSetpoint(shootPhaseEnum shootPhase, shootTypeEnum shootType, Position2D pos)
{
    std::ostringstream str;
    str << "shootPhase=" << static_cast<std::underlying_type<actionTypeEnum>::type>(shootPhase) << "; shootType=" << static_cast<std::underlying_type<actionTypeEnum>::type>(shootType) << "; pos=" << pos.tostr();
    TRACE_FUNCTION(str.str().c_str());

    T_SHOOT_SETPOINT shootSetpoint;

    shootSetpoint.shootPhase = shootPhase;
    shootSetpoint.shootType = shootType;
    shootSetpoint.position.x = pos.x;
    shootSetpoint.position.y = pos.y;
    shootSetpoint.position.z = pos.phi;

    tprintf("             put SHOOT_SETPOINT phase=%s type=%s pos=[%6.2f, %6.2f, %6.2f]", enum2str(shootPhase), enum2str(shootType), pos.x, pos.y, pos.phi);
    _rtdb->put(SHOOT_SETPOINT, &shootSetpoint);
}

void MP_RTDBOutputAdapter::setKickerPower(float kickerPower)
{
    TRACE_FUNCTION("");

    T_KICKER_SETPOINT kickerSetpoint;
    kickerSetpoint.kickerSetpointType = kickerSetpointTypeEnum::SHOOT;
    kickerSetpoint.kickerHeight = 0.0; // unused
    kickerSetpoint.kickerPower = kickerPower;

    setKickerSetpoint(kickerSetpoint);
}

void MP_RTDBOutputAdapter::setKickerHeight(float kickerHeight)
{

    TRACE_FUNCTION("");

    T_KICKER_SETPOINT kickerSetpoint;
    kickerSetpoint.kickerSetpointType = kickerSetpointTypeEnum::SET_HEIGHT;
    kickerSetpoint.kickerHeight = kickerHeight;
    kickerSetpoint.kickerPower = 0.0; // unused

    setKickerSetpoint(kickerSetpoint);
}

void MP_RTDBOutputAdapter::setKickerSetpoint(T_KICKER_SETPOINT const &kickerSetpoint)
{
    TRACE_FUNCTION("");
    tprintf("             put KICKER_SETPOINT type=%s power=%6.2f height=%6.2f", enum2str(kickerSetpoint.kickerSetpointType), kickerSetpoint.kickerPower, kickerSetpoint.kickerHeight);
    _rtdb->put(KICKER_SETPOINT, &kickerSetpoint);
}

void MP_RTDBOutputAdapter::setBallHandlersSetpoint(bool enabled)
{
    std::stringstream str;
    str << "enabled=" << enabled;
    TRACE_FUNCTION(str.str().c_str());

    T_BALLHANDLERS_SETPOINT ballHandlersSetpoint;
    ballHandlersSetpoint = enabled;

    tprintf("             put BALLHANDLERS_SETPOINT %d", enabled);
    _rtdb->put(BALLHANDLERS_SETPOINT, &ballHandlersSetpoint);
}

void MP_RTDBOutputAdapter::setActionResult(T_ACTION_RESULT mpActionResult)
{
    TRACE_FUNCTION("");

    tprintf("             put ACTION_RESULT result=%s", enum2str(mpActionResult.result));

    _rtdb->put(ACTION_RESULT, &mpActionResult);
}

void MP_RTDBOutputAdapter::setKeeperFrameSetpoint(T_KEEPERFRAME_SETPOINT s)
{
    tprintf("put KEEPERFRAME %s", enum2str(s));
    _rtdb->put(KEEPERFRAME_SETPOINT, &s);
}

void MP_RTDBOutputAdapter::disableObstacleAvoidance()
{
    T_CONFIG_PATHPLANNING config;
    int r = 0, ageMs = 0;
    r = _rtdb->get(CONFIG_PATHPLANNING, &config, ageMs, _myRobotId);
    if (r == RTDB2_SUCCESS)
    {
        config.obstacleAvoidance.enabled = false;
        tprintf("disabling obstacle avoidance");
        // note: there is a small race condition between previous get and this put,
        // but does not need to be addressed since changing configuration is a very irregular event
        _rtdb->put(CONFIG_PATHPLANNING, &config);
    }
}


//
//void cRTDBOutputAdapter::setObstacles()
//{
//    TRACE_FUNCTION("");
//    std::vector<obstacleClass_t> obstacles;
//    _obstacleAdmin->getObstacles(obstacles);
//
//    T_OBSTACLES obstacleResults;
//    for (auto it = obstacles.begin(); it != obstacles.end(); it++)
//    {
//        obstacleResult obst;
//        obst.position.x = it->getX();
//        obst.position.y = it->getY();
//        obst.velocity.x = it->getVX();
//        obst.velocity.y = it->getVY();
//        obst.confidence = it->getConfidence();
//        obst.id = 0; // TODO fill in, depending on if there is a use case from teamplay?
//        obstacleResults.push_back(obst);
//    }
//
//    _rtdb->put(OBSTACLES, &obstacleResults);
//}
