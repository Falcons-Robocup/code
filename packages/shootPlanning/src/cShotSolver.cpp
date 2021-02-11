// Copyright 2017-2019 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cShotSolver.cpp
 *
 *  Created on: Dec 17, 2017
 *      Author: Jan Feitsma
 */


#include "int/cShotSolver.hpp"
#include "tracing.hpp"
#include "cDiagnostics.hpp"

#include <cmath>
#include <cassert>

#define DISTANCE_RESOLUTION (1e-2)
#define STRENGTH_RESOLUTION (1e-2)
#define ANGLE_RESOLUTION (1e-2)
#define Z_RESOLUTION (1e-2)

const float initial_position = 0.3; // distance between the center of the ball (in ball handler position) and the center of the robot
const float ball_radius = 0.11; // radius of a ball

cShotSolver::cShotSolver()
{
}

cShotSolver::~cShotSolver()
{
}

void cShotSolver::load(std::string const &filename)
{
    _calibration.clear();
    _calibration.load(filename);
    selectShots();
}

void cShotSolver::selectShots()
{
    std::vector<std::tuple<float, float, float, float>> allMeasurementSamples; // height, power, maxZ, distance
    _calibration.get(allMeasurementSamples);

    for (auto it = allMeasurementSamples.begin(); it != allMeasurementSamples.end(); ++it)
    {
        float angle = std::get<0>(*it);
        float power = std::get<1>(*it);
        float height = std::get<2>(*it);
        float distance = std::get<3>(*it);

        if ( height >= ball_radius && distance >= initial_position) // sanity check
        {
            if (angle > 179)
            {
                // power -> (max_z, max_distance) , angle = 180
                _lookupTblLob[power] = std::tuple<float, float>(height, distance);
            }

            if (power > 179)
            {
                //  angle -> (max_z, max_distance) , power = 180
                _lookupTblStraight[angle] = std::tuple<float, float>(height, distance);
            }
        }
    }
}

bool cShotSolver::query(float distance, float preferredHeight, bool doLob, float &resultStrength, float &resultAngle)
{
    TRACE_FUNCTION("");
    std::map<float,std::tuple<float, float>> & lookupTable = _lookupTblStraight;
    float result = 180;
    float lowerBound = -999;
    float upperBound = -999;
    float lowerResult = 0;
    float upperResult = 0;
    bool r = true;

    if (doLob)
    {
        lookupTable = _lookupTblLob;
    }

    // loop through table
    for (std::map<float,std::tuple<float, float>>::iterator it=lookupTable.begin(); it!=lookupTable.end(); ++it)
    {
        float dmax = std::get<1>(it->second);
        float zmax = std::get<0>(it->second);

        // calculate intersection
        float y = - pow(2.0 / (dmax - initial_position), 2) * pow((distance - initial_position) - (dmax - initial_position)/2.0, 2) * (zmax - ball_radius) + zmax ;

        if ( preferredHeight <= y)
        {
            upperBound = y;
            upperResult = it->first;
            break;
        }
        else
        {
            if (y >= lowerBound )
            {
                lowerBound = y;
                lowerResult = it->first;
            }
            else
            {
                // don't break since the calibration file can have non precise values
                // which causes premature stop
    
            }
        }
    }
    
    //TRACE("input (distance=%.3f z=%.3f doLob=%d) ==> result (lowerResult=%.3f upperResult=%.3f), ( %.3f <= %.3f <= %.3f)", 
    //                  distance, preferredHeight, doLob, lowerResult, upperResult, lowerBound, preferredHeight, upperBound);

    if ((upperBound != -999) && (lowerBound != -999)) 
    {
        /* interpolate */
        if (upperBound == lowerBound)
        {
            /* Should never happen */
            result = lowerResult;
        }
        else
        {
            result = lowerResult + (upperResult - lowerResult) * (preferredHeight - lowerBound) / (upperBound - lowerBound);
        } 

    }
    else
    {  
        if (upperBound == -999)
        {
            // No match found, the closest shoot is selected 
            // (The ball should bounce before reaching the goal)
            result = lowerResult;
        }
        else // (lowerBound == -999)
        {
            // No match found, the closest shoot is selected 
            // Should not happen (with a good calibration file)
            result = upperResult;
        }

        if (result == -999)
        {
            /* Should never happen (with non empty calibration file) */  
            result = 180;
            r = false;
        }
    }
    
    if (doLob)
    {
        resultAngle = 180;
        resultStrength = result;
    }
    else
    {
        resultAngle = result;
        resultStrength = 180;
    }
    
    TRACE("input (distance=%.3f z=%.3f doLob=%d) ==> result (strength=%.3f angle=%.3f)", distance, preferredHeight, doLob, resultStrength, resultAngle);
    tprintf("input (distance=%.3f z=%.3f doLob=%d) ==> result (strength=%.3f angle=%.3f)", distance, preferredHeight, doLob, resultStrength, resultAngle);
    return r;
}

