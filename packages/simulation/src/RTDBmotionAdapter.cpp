 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBMotionAdapter.cpp
 *
 *  Created on: Feb 12, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBmotionAdapter.hpp"

#include "int/generated_enum2str.hpp"
#include "int/RTDBaccess.hpp"
#include "int/simulationCapabilities.hpp"

#include <stdexcept>

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"


Velocity2D RTDBMotionAdapter::getVelocity (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_ROBOT_VELOCITY_SETPOINT robotVelocitySetpoint;
    int ageMs = 0;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(ROBOT_VELOCITY_SETPOINT, &robotVelocitySetpoint, ageMs, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting ROBOT_VELOCITY_SETPOINT from RtDB");
    }
    if (ageMs >= MAXIMUM_AGE_MS)
    {
        //throw std::runtime_error("ROBOT_VELOCITY_SETPOINT data from RtDB is too old");
        // with current execution architecture, it is very possible / likely for setpoint to grow old
        // so we should not throw an error, instead apply a kind of inertia/watchdog: zero velocity
        // (otherwise, behavior in simulation would be that robot would continue to drift off forever with its last small nonzero setpoint
        robotVelocitySetpoint.x = 0.0;
        robotVelocitySetpoint.y = 0.0;
        robotVelocitySetpoint.Rz = 0.0;
    }
    tprintf("get ROBOT_VELOCITY_SETPOINT [%s %s] robotVelocity=[%6.2f, %6.2f, %6.2f]",
                      enum2str(teamID), enum2str(robotID),
                      robotVelocitySetpoint.x,
                      robotVelocitySetpoint.y,
                      robotVelocitySetpoint.Rz);
    return Velocity2D(robotVelocitySetpoint.x,
                      robotVelocitySetpoint.y,
                      robotVelocitySetpoint.Rz);
}


Kicker RTDBMotionAdapter::getKickerData (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_KICKER_SETPOINT kickerSetpoint;
    int ageMs = 0;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(KICKER_SETPOINT, &kickerSetpoint, ageMs, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting KICKER_SETPOINT from RtDB");
    }
    if (ageMs >= MAXIMUM_AGE_MS)
    {
        throw std::runtime_error("KICKER_SETPOINT data from RtDB is too old");
    }
    tprintf("get KICKER_SETPOINT [%s %s] kickerHeight=%6.2f kickerPower=%6.2f",
            enum2str(teamID), enum2str(robotID),
            kickerSetpoint.kickerHeight, kickerSetpoint.kickerPower);
    return {kickerSetpoint.kickerHeight, kickerSetpoint.kickerPower};
}


bool RTDBMotionAdapter::hasBallHandlersEnabled (const TeamID& teamID, const RobotID& robotID) const
{
    TRACE_FUNCTION("");
    T_BALLHANDLERS_SETPOINT ballhandlersSetpoint;
    int ageMs = 0;
    auto robotNumber = getRobotNumber(robotID);
    auto rtdbConnection = getRTDBConnection(teamID, robotID);
    auto r = rtdbConnection->get(BALLHANDLERS_SETPOINT, &ballhandlersSetpoint, ageMs, robotNumber);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting BALLHANDLERS_SETPOINT from RtDB");
    }
    if (ageMs >= MAXIMUM_AGE_MS)
    {
        throw std::runtime_error("BALLHANDLERS_SETPOINT data from RtDB is too old");
    }
    tprintf("get BALLHANDLERS_SETPOINT [%s %s] enabled=%s",
            enum2str(teamID), enum2str(robotID),
            ((ballhandlersSetpoint)?("Yes"):("No ")));
    return ballhandlersSetpoint;
}
