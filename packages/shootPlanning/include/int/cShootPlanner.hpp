 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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

#include "falconsCommon.hpp"
#include "shootTypeEnum.hpp"
#include "AbstractInterpolator.hpp" // from package 'filters'

#include "int/adapters/cRTDBOutputAdapter.hpp"
#include "int/cShotSolver.hpp"

typedef struct
{
    bool alwaysLobshot;
    bool alwaysStraightshot;
    double shotStrengthScaling;
    double shotAngleScaling;
    double lobAngleScaling;
    double passScaling;
    double minLobDistance;
    double maxLobDistance;
} shootPlanningParams_t;

class cShootPlanner
{
    public:
        cShootPlanner();
        ~cShootPlanner();

        void prepareForShot(float distance, float z, shootTypeEnum shootType);
        void executeShot();
        void executePass(float distance);

    private:
        
        // models
        bool canPrepare();
        void calculatePassPower(float distance);
        void calculateLob(float distance, float z);
        void calculateStraightShot(float distance, float z);

        T_CONFIG_SHOOTPLANNING getConfig();

        AbstractInterpolator *_passInterpolator;
        cShotSolver *_shotSolver;

        cRTDBOutputAdapter _rtdbOutputAdapter;
        float _kickerStrength;
        float _kickerAngle;
        double _lastSetHeightTimestamp;
};

#endif /* CSHOOTPLANNER_HPP_ */
