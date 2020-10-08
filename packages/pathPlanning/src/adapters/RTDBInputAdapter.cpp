 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBInputAdapter.cpp
 *
 *  Created on: Oct 27, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/RTDBInputAdapter.hpp"

#include "falconsCommon.hpp" //getRobotNumber(), getTeamChar()
#include "cDiagnostics.hpp"
#include "tracing.hpp"

RTDBInputAdapter::RTDBInputAdapter()
{
    TRACE_FUNCTION("");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    _wmClient = new cWorldModelClient();
}

RTDBInputAdapter::~RTDBInputAdapter()
{
    TRACE_FUNCTION("");
}

void RTDBInputAdapter::waitForMotionSetpoint()
{
    TRACE_FUNCTION("");
    _rtdb->waitForPut(MOTION_SETPOINT);
}

T_MOTION_SETPOINT RTDBInputAdapter::getMotionSetpoint()
{
    return _motionSetpoint;
}

std::vector<forbiddenArea> RTDBInputAdapter::getForbiddenAreas()
{
    return _forbiddenAreas;
}

robotState RTDBInputAdapter::getRobotState()
{
    return _robot;
}

std::vector<robotState> RTDBInputAdapter::getTeamMembers()
{
    return _teamMembers;
}

std::vector<obstacleResult> RTDBInputAdapter::getObstacles()
{
    return _obstacles;
}

std::vector<ballResult> RTDBInputAdapter::getBalls()
{
    return _balls;
}

void RTDBInputAdapter::fetch()
{
    TRACE_FUNCTION("");

    int r = 0;

    // motionSetpoint
    r = _rtdb->get(MOTION_SETPOINT, &_motionSetpoint);
    if (r == RTDB2_SUCCESS)
    {
        tprintf("get MOTION_SETPOINT action=%s pos=[%6.2f, %6.2f, %6.2f] slow=%d", enum2str(_motionSetpoint.action), _motionSetpoint.position.x, _motionSetpoint.position.y, _motionSetpoint.position.z, _motionSetpoint.slow);
    }
    else
    {
        TRACE_ERROR("failed to get MOTION_SETPOINT");
    }

    // poke worldModel to update
    _wmClient->update();

    // store own robot position and velocity
    r = _wmClient->getRobotState(_robot, _myRobotId);
    if (r == false)
    {
        TRACE_ERROR("failed to get robotState from worldModel");
    }

    // get obstacles, balls and teammembers
    _obstacles = _wmClient->getObstacles(); // opponents actually
    _balls = _wmClient->getBalls();
    _teamMembers = _wmClient->getTeamMembersExcludingSelf();

    // get external forbidden areas
    T_FORBIDDEN_AREAS tmpForbiddenAreas;
    r = _rtdb->get(FORBIDDEN_AREAS, &tmpForbiddenAreas); // it may happen that this data is not filled (yet)
    if (r == RTDB2_SUCCESS)
    {
        _forbiddenAreas = tmpForbiddenAreas;
        TRACE("fetched %d forbidden areas", (int)tmpForbiddenAreas.size());
    }
    else
    {
        TRACE("clearing forbidden areas");
        _forbiddenAreas.clear();
    }
}

