 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cShootPlanner.cpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */

#include "int/cShootPlanner.hpp"
#include "int/cLinearInterpolator.hpp"
#include "cDiagnostics.hpp"
#include "FalconsCommon.h"
#include "tracing.hpp"

using std::runtime_error;
using std::exception;

cShootPlanner::cShootPlanner(shootPlanningParams_t spParams)
{
    _spParams = spParams;
    _lastSetHeightTimestamp = 0.0;
    _kickerAngle = 30;
    _kickerStrength = 180;

    _passInterpolator = new cLinearInterpolator();
    _passInterpolator->load(determineConfig("shootPlanningPassCalibration", ".txt"));
    
    TRACE("constructing lookup table");
    _shotSolver = new cShotSolver();
    _shotSolver->load(determineConfig("shootPlanningShotCalibration", ".txt"));
    
    TRACE("cShootPlanner constructed with alwaysLobshot=%d", _spParams.alwaysLobshot);

}

cShootPlanner::~cShootPlanner()
{

}

void cShootPlanner::prepareForShot(float distance, float z, shootTypeEnum shootType)
{
    std::ostringstream ss;
    ss << "dist=" << distance << " z=" << z << " shootType=" << (int) shootType;
    TRACE_FUNCTION(ss.str().c_str());
    // calculate kicker settings
    if (canPrepare())
    {
        if (shootType == shootTypeEnum::PASS)
        {
            _kickerAngle = 0.0;  
        }
        else if ( _spParams.alwaysLobshot || 
            ( shootType == shootTypeEnum::LOB && 
                (distance <= _spParams.maxLobDistance) && 
                (distance >= _spParams.minLobDistance) && 
                !_spParams.alwaysStraightshot))
        {
            calculateLob(distance, z);
        }
        else
        {
            calculateStraightShot(distance, z);
        }
        TRACE("setHeightKicker %.2f", _kickerAngle);
        tprintf("setHeightKicker %.2f", _kickerAngle);
        _rtdbOutputAdapter.setKickerSetpoint(kickerSetpointTypeEnum::SET_HEIGHT, _kickerAngle, 0.0);
        _lastSetHeightTimestamp = rtime::now();
    }
}

void cShootPlanner::executeShot()
{
    TRACE_FUNCTION("");
    // shoot with power which was prepared
    _rtdbOutputAdapter.setKickerSetpoint(kickerSetpointTypeEnum::SHOOT, _kickerAngle, _kickerStrength);
}

void cShootPlanner::executePass(float distance)
{
    TRACE_FUNCTION("");
    try
    {
        // calculate pass power based on given distance
        calculatePassPower(distance);
        
        // assume kicker was reset after last lobshot
        // otherwise we might be too late here (kicker still at nonzer), perhaps need preparePass?

        _rtdbOutputAdapter.setKickerSetpoint(kickerSetpointTypeEnum::SHOOT, 0.0, _kickerStrength);

    } catch (exception &e)
    {
        throw e;
    }
}

bool cShootPlanner::canPrepare()
{
    // use a brief timer here to prevent 30Hz spam of slightly changing consecutive height setpoints
    // during rotation preparation for shot
    // which has seen to cause ioBoard reconnecting and even high-pitch noises
    double t = rtime::now();
    if ((t - _lastSetHeightTimestamp) > 0.5)
    {
        return true;
    }
    return false;
}

void cShootPlanner::calculatePassPower(float distance)
{
    _kickerStrength = _spParams.passScaling * _passInterpolator->interpolate(distance);
}

void cShootPlanner::calculateLob(float distance, float z)
{
    float tmpStrength = 0.0;
    float tmpAngle = 0.0;

    bool r = _shotSolver->query(distance, z, true, tmpStrength, tmpAngle);

    _kickerAngle = tmpAngle * _spParams.lobAngleScaling;
    _kickerStrength = tmpStrength * _spParams.shotStrengthScaling;
    
    if (!r)
    {
        TRACE_WARNING("failed to determine lob shot settings (distance=%.6f z=%.6f), fallback to defaults", distance, z);
    }
}

void cShootPlanner::calculateStraightShot(float distance, float z)
{
    /* Even if z = 0, set the right Angle to avoid friction */
    float tmpStrength = 0.0;
    float tmpAngle = 0.0;
    bool r = _shotSolver->query(distance, z, false, tmpStrength, tmpAngle);
    
    _kickerAngle = tmpAngle * _spParams.shotAngleScaling;
    _kickerStrength = tmpStrength * _spParams.shotStrengthScaling;
    
    if (!r)
    {
        TRACE_WARNING("failed to determine straight shot settings (distance=%.6f z=%.6f), fallback to defaults", distance, z);
    }
}
