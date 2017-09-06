 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cShootPlanner.hpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Thomas Kerkhof
 */

#ifndef CSHOOTPLANNER_HPP_
#define CSHOOTPLANNER_HPP_

#include "FalconsCommon.h"

#include "int/cWorldModelAdapter.hpp"
#include "int/cKickerAdapter.hpp"
#include "int/cBallHandlerAdapter.hpp"
#include "int/cReconfigureAdapter.hpp"


typedef struct
{
  bool alwaysLobshot;
  bool disableBHBeforeShot;
  bool calibrationMode;
  double setHeightDelay;
  double disableBHDelay;
  double lobshotScaling;
  double min_lob_distance;
  double max_lob_distance;
  double max_shot_power;
  double max_lever_angle;
} shootPlanningParams_t;

typedef struct
{
  double x;
  double y;
  double strength;
} passBall_t;

typedef struct
{
  double strength;
} selfPass_t;

typedef struct
{
  double x;
  double y;
  double z;
  double strength;
} shootAtTarget_t;

typedef struct
{
  double x;
  double y;
  double z;
  double strength;
} lobshotTarget_t;

typedef struct
{
  double ls_c1;
  double ls_c2;
  double ls_c3;
  double ls_c4;
  double ls_c5;
  double ls_c6;
} lobshotFuncParams_t;

typedef struct
{
  double p_c1;
  double p_c2;
  double p_c3;
  double p_c4;
  double p_c5;
  double p_c6;
} passFuncParams_t;

typedef struct
{
  double ls_pwr10m;
  double ls_pwr9m;
  double ls_pwr8m;
  double ls_pwr7m;
  double ls_pwr6m;
  double ls_pwr5m;
  double ls_pwr4m;
} lobshotPowers_t;

typedef struct
{
  double p_pwr10m;
  double p_pwr9m;
  double p_pwr8m;
  double p_pwr7m;
  double p_pwr6m;
  double p_pwr5m;
  double p_pwr4m;
  double p_pwr3m;
  double p_pwr2m;
  double p_pwr1m;
} passPowers_t;

class cShootPlanner
{
    public:
        cShootPlanner(const shootPlanningParams_t spParams, const lobshotPowers_t lsPowers, const lobshotFuncParams_t lsFuncParams,
        		const passPowers_t pPowers, const passFuncParams_t pFuncParams);

        ~cShootPlanner();

        void activateShootPlanning();
        void deactivateShootPlanning();
        bool isShootPlanningActive();

        void passBall(const passBall_t passParams);
        void selfPass(const selfPass_t selfPassParams);
        void shootAtTarget(const shootAtTarget_t shootParams);
        void lobShot(const lobshotTarget_t shootParams);
        void lobCalculatorPoints ( double distance, double &kick_angle, double &kick_strength);
        void lobCalculatorFunction ( double distance,  double &kick_angle,  double &kick_strength);

    private:
        void delay(double seconds);
        bool _isShootPlanningActive;

        cWorldModelAdapter _wmAdapter;
        cKickerAdapter _kickAdapter;
        cBallHandlerAdapter _bhAdapter;
        cReconfigureAdapter _reconfigureAdapter;
        shootPlanningParams_t _spParams;
        lobshotPowers_t _lsPowers;
        lobshotFuncParams_t _lsFuncParams;
        passPowers_t _pPowers;
        passFuncParams_t _pFuncParams;
};

#endif /* CSHOOTPLANNER_HPP_ */
