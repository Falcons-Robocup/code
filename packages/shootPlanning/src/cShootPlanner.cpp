 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
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
#include <cDiagnosticsEvents.hpp>
#include "FalconsCommon.h"

using std::runtime_error;
using std::exception;

cShootPlanner::cShootPlanner(shootPlanningParams_t spParams , lobshotPowers_t lsPowers, lobshotFuncParams_t lsFuncParams,
								passPowers_t pPowers, passFuncParams_t pFuncParams)
{
	_spParams = spParams;
	_lsPowers = lsPowers;
	_lsFuncParams = lsFuncParams;
	_pPowers = pPowers;
	_pFuncParams = pFuncParams;
	_isShootPlanningActive = true;

    TRACE("cShootPlanner constructed with alwaysLobshot=%d disableBH=%d _disableBHDelay=%6.2f setheightDelayDelay=%6.2f",
    		_spParams.alwaysLobshot, _spParams.disableBHBeforeShot, _spParams.setHeightDelay ,_spParams.disableBHDelay);

}

cShootPlanner::~cShootPlanner()
{

}

void cShootPlanner::activateShootPlanning()
{
    _isShootPlanningActive = true;
    TRACE("shootplanning activated");
}

void cShootPlanner::deactivateShootPlanning()
{
    _isShootPlanningActive = false;
    TRACE("shootplanning de-activated");
}

bool cShootPlanner::isShootPlanningActive()
{
    return (_isShootPlanningActive && _wmAdapter.isRobotActive());
}

void cShootPlanner::passBall(const passBall_t passParams)
{
    bool heightSuccessful = false;
    bool shootSuccessful = false;
    bool bhSuccessful = false;

    try
    {
        if (isShootPlanningActive())
        {
            TRACE("pass (x=%.1f, y=%.1f, strength=%.1f, disableBH=%d)",
            		passParams.x, passParams.y, passParams.strength,_spParams.disableBHBeforeShot);
            
            /* Adjust the height of the kick solenoid */
            _kickAdapter.setHeightKicker(0.0, heightSuccessful);

            /* Shoot */
            if (_spParams.disableBHBeforeShot)
            {
            	_bhAdapter.disableBH(bhSuccessful);
            	delay(_spParams.disableBHDelay);
            	_kickAdapter.kickBall(passParams.strength, shootSuccessful);
            	_bhAdapter.enableBH(bhSuccessful);            }
            else
            {
            	_kickAdapter.kickBall(passParams.strength, shootSuccessful);
            }
        }
        else
        {
            TRACE_ERROR("Ignored passBall since we are not active");
        }
    } catch (exception &e)
    {
        throw e;
    }
}

void cShootPlanner::selfPass(const selfPass_t selfPassParams)
{
    bool heightSuccessful = false;
    bool shootSuccessful = false;

    try
    {
        if (isShootPlanningActive())
        {
            TRACE("selfpass (strength=%.1f, disableBH=%d)", selfPassParams.strength,_spParams.disableBHBeforeShot);

            /* Adjust the height of the kick solenoid */
            _kickAdapter.setHeightKicker(0.0, heightSuccessful);
            /* Shoot */
           	_kickAdapter.kickBall(selfPassParams.strength, shootSuccessful);
        }
        else
        {
            TRACE_ERROR("Ignored selfPass since we are not active");
        }
    } catch (exception &e)
    {
        throw e;
    }
}

void cShootPlanner::shootAtTarget(const shootAtTarget_t shootParams)
{
    double kick_angle = 0.0;
    bool heightSuccessful = false;
    bool shootSuccessful = false;
    bool bhSuccessful = false;

    try
    {
    	if (_spParams.alwaysLobshot && (shootParams.strength > 150)) //Also check for shootstrength untill teamplay uses the pass service for a pass
    	{
    		TRACE_INFO("Always Lobshot enabled");
    		lobshotTarget_t lobshotParams;
    		lobshotParams.strength = shootParams.strength;
    		lobshotParams.x = shootParams.x;
    		lobshotParams.y = shootParams.y;
    		lobshotParams.z = shootParams.z;

    		lobShot(lobshotParams);
    	}else if (isShootPlanningActive())
        {
            TRACE_INFO("shootAtTarget (x=%.1f, y=%.1f, z=%.1f, strength=%.1f, disableBH=%d)",
            		shootParams.x, shootParams.y, shootParams.z, shootParams.strength, _spParams.disableBHBeforeShot);

            if (shootParams.z <= 0.1) //if the target height is 10cm or lower
            {
                kick_angle = 0.0; //shoot straight over the floor
            }
            else
            {
                // determine angles to shoot straight in the crossing for example. Not implemented yet, so 0
                kick_angle = 0.0;
            }

            /* Adjust the height of the kick solenoid */
            _kickAdapter.setHeightKicker(kick_angle, heightSuccessful);

            /* Shoot */
            if (_spParams.disableBHBeforeShot)
            {
            	_bhAdapter.disableBH(bhSuccessful);
            	delay(_spParams.disableBHDelay);
            	_kickAdapter.kickBall(shootParams.strength, shootSuccessful);
            	_bhAdapter.enableBH(bhSuccessful);
            }
            else
            {
            	_kickAdapter.kickBall(shootParams.strength, shootSuccessful);
            }
        }
        else
        {
            TRACE_ERROR("Ignored shootAtTarget since we are not active");
        }
    } catch (exception &e)
    {
        throw e;
    }
}

void cShootPlanner::lobShot(const lobshotTarget_t shootParams)
{
	//Lob shot: Basic function:
	//Check if we are not too close to the goal or too far away, then determine angle and strength. Checks can be removed when implemented in teamplay.
    Position2D currPos = Position2D();
    double kick_distance = 0.0;
    double delta_x = 0.0;
    double delta_y = 0.0;
    bool heightSuccessful = false;
    bool shootSuccessful = false;
    bool bhSuccessful = false;

    double kick_angle = _spParams.max_lever_angle;
    //Default Kick power
    double kick_strength = _spParams.max_shot_power;

    //Override Opponent Goal for now. Only used for distance, not for aiming
    //Can be removed if target is received from teamplay
    double target_x = 0;
    double target_y = 9;

    try
    {
        if (isShootPlanningActive())
        {
            TRACE("lobShot (x=%.1f, y=%.1f, z=%.1f, strength=%.1f, disableBH=%d)",
            		shootParams.x, shootParams.y, shootParams.z, shootParams.strength,_spParams.disableBHBeforeShot);

            /* Fetch current position to calculate wanted angle of the robot */
            _wmAdapter.getPosition(currPos);

            ///* Calculate the kick distance */
            delta_x = target_x - currPos.x;
            delta_y = target_y - currPos.y;
            kick_distance = sqrt(pow(delta_x, 2.0) + pow(delta_y, 2.0));

            if ((kick_distance < _spParams.min_lob_distance) || (kick_distance > _spParams.max_lob_distance))
            {
                kick_angle = 0.0;
                TRACE_INFO("Lobshot not possible, shooting straight");
            }
            else
            {
            	if (!_spParams.calibrationMode){//Use the function to determine the strength
            		lobCalculatorFunction(kick_distance,kick_angle, kick_strength);
            	}else {//Use the calibration function
            		lobCalculatorPoints(kick_distance,kick_angle, kick_strength);
            	}
            	TRACE_INFO("Lobshot: dist:%6.2f, pwr:%6.2f, angle:%6.2f", kick_distance, kick_strength, kick_angle);
            }


            /* Shoot */
            if (_spParams.disableBHBeforeShot)
            {
            	_kickAdapter.setHeightKicker(kick_angle, heightSuccessful);
            	delay(_spParams.setHeightDelay); //extra delay if the lever is not fast enough. Can be 0 for a fast lever adjustment
            	_bhAdapter.disableBH(bhSuccessful);
            	delay(_spParams.disableBHDelay);
            	_kickAdapter.kickBall(kick_strength, shootSuccessful);
            	delay(_spParams.disableBHDelay); // if the delay was too short the enable was unsuccessfull (typically < 0.2s)
            	_bhAdapter.enableBH(bhSuccessful);
            }

            else
            {
            	_kickAdapter.setHeightKicker(kick_angle, heightSuccessful);
                TRACE("Sleep for SetHeight");
            	delay(_spParams.setHeightDelay);
            	_kickAdapter.kickBall(kick_strength, shootSuccessful);
            }
            //reset height to 0
            _kickAdapter.setHeightKicker(0.0, heightSuccessful);
        }
        else
        {
            TRACE_ERROR("Ignored lobshot since we are not active");
        }
    } catch (exception &e)
    {
        throw e;
    }
}

void cShootPlanner::delay(double seconds)
{
	int delayus = int(seconds*1000000.0);
    usleep(delayus);
}

void cShootPlanner::lobCalculatorPoints (double kick_distance, double &kick_angle, double &kick_strength)
{
	//This function is used for calibration purposes.
	//For longer distances the lever is lifted abit (TODO: not yet variable)
		double dist4 = 4.0;
	    double dist5 = 5.0;
	    double dist6 = 6.0;
	    double dist7 = 7.0;
	    double dist8 = 8.0;
	    double dist9 = 9.0;
	    double dist10 = 10.0;

	    double pwr4 = _lsPowers.ls_pwr4m;//87
	    double pwr5 = _lsPowers.ls_pwr5m;//100
	    double pwr6 = _lsPowers.ls_pwr6m;//117
	    double pwr7 = _lsPowers.ls_pwr7m;//140
	    double pwr8 = _lsPowers.ls_pwr8m;//170
	    double pwr9 = _lsPowers.ls_pwr9m;//180
	    double pwr10 = _lsPowers.ls_pwr10m;//180

	    if (kick_distance <= dist4){
			kick_strength = pwr4;
		}else if (kick_distance <= dist5){
			kick_strength = pwr5;
		}else if (kick_distance <= dist6){
			kick_strength = pwr6;
		}else if (kick_distance <= dist7){
			kick_strength = pwr7;
		}else if (kick_distance <= dist8){
			kick_strength = pwr8;
		}else if (kick_distance <= dist9){
			kick_strength = pwr9;
			kick_angle= kick_angle*0.8;//TODO: Make variable and smart, now limited to not bounce over the goal
		}else if (kick_distance <= dist10){
			kick_strength = pwr10;
			kick_angle=kick_angle*0.7;//TODO: Make variable and smart, now limited to not bounce over the goal
		}
		kick_strength = kick_strength * _spParams.lobshotScaling;

}

void cShootPlanner::lobCalculatorFunction ( double x,  double &kick_angle,  double &kick_strength)
{
    //5th order poly gave the best fit for these calibration constants
	//If the calibration points change, you first need to visiualize the function to see the fit.
    // function coefficients (Calculated online for dist/pwr points)
	//y = c1*x^5 + c2*x^4 + c3*x^3 + c4*x^2 + c5*x + c6

	kick_strength = (_lsFuncParams.ls_c1 * pow(x,5) +
					_lsFuncParams.ls_c2 * pow(x,4)+
					_lsFuncParams.ls_c3 * pow(x,3)+
					_lsFuncParams.ls_c4 * pow(x,2)+
					_lsFuncParams.ls_c5 * pow(x,1)+
					_lsFuncParams.ls_c6);

	kick_strength = kick_strength * _spParams.lobshotScaling;

	//To prevent the ball from bouncing over the goal the lever has to be lifted a bit for larger distances
	if (x >= 9){
		kick_angle= kick_angle*0.8; //TODO: Make variable and smart, now limited to not bounce over the goal
	}else if (x >= 10){
		kick_angle= kick_angle*0.7;//TODO: Make variable and smart, now limited to not bounce over the goal
	}
}
