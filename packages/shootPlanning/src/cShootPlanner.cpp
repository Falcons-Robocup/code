// Copyright 2015-2021 Erik Kouters (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShootPlanner.cpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */
#include "FalconsRTDB.hpp"
#include "ConfigRTDBAdapter.hpp"

#include "int/cShootPlanner.hpp"
#include "LinearInterpolator.hpp" // from package 'filters'
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"

using std::runtime_error;
using std::exception;

ConfigRTDBAdapter<T_CONFIG_SHOOTPLANNING>* _configAdapter;

cShootPlanner::cShootPlanner()
{
    // Fetch configuration from yaml file
    std::string configFile = determineConfig("ShootPlanning");
    _configAdapter = new ConfigRTDBAdapter<T_CONFIG_SHOOTPLANNING>(CONFIG_SHOOTPLANNING);
    _configAdapter->loadYAML(configFile);

    _lastSetHeightTimestamp = 0.0;
    _kickerAngle = 30;
    _kickerStrength = 180;

    _passInterpolator = new LinearInterpolator();
    _passInterpolator->load(determineConfig("shootPlanningPassCalibration", ".txt"));

    TRACE("constructing lookup table");
    _shotSolver = new cShotSolver();
    _shotSolver->load(determineConfig("shootPlanningShotCalibration", ".txt"));
    
    TRACE("cShootPlanner constructed");

}

cShootPlanner::~cShootPlanner()
{
    delete _configAdapter;
    delete _passInterpolator;
    delete _shotSolver;
}

T_CONFIG_SHOOTPLANNING cShootPlanner::getConfig()
{
    T_CONFIG_SHOOTPLANNING result;
    _configAdapter->get(result);
    return result;
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
        else if ( getConfig().alwaysLobshot || 
            ( shootType == shootTypeEnum::LOB && 
                (distance <= getConfig().maxLobDistance) && 
                (distance >= getConfig().minLobDistance) && 
                !getConfig().alwaysStraightshot))
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
        _lastSetHeightTimestamp = ftime::now();
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
    double t = ftime::now();
    if ((t - _lastSetHeightTimestamp) > 0.5)
    {
        return true;
    }
    return false;
}

void cShootPlanner::calculatePassPower(float distance)
{
    _kickerStrength = getConfig().passScaling * _passInterpolator->evaluate(distance);
}

void cShootPlanner::calculateLob(float distance, float z)
{
    float tmpStrength = 0.0;
    float tmpAngle = 0.0;

    bool r = _shotSolver->query(distance, z, true, tmpStrength, tmpAngle);

    _kickerAngle = tmpAngle * getConfig().lobAngleScaling;
    _kickerStrength = tmpStrength * getConfig().shotStrengthScaling;
    
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
    
    _kickerAngle = tmpAngle * getConfig().shotAngleScaling;
    _kickerStrength = tmpStrength * getConfig().shotStrengthScaling;
    
    if (!r)
    {
        TRACE_WARNING("failed to determine straight shot settings (distance=%.6f z=%.6f), fallback to defaults", distance, z);
    }
}
