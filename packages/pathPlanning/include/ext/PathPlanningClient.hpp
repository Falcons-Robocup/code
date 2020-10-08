 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * PathPlanningClient.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNINGCLIENT_HPP_
#define PATHPLANNINGCLIENT_HPP_


#include "actionResult.hpp" // sharedTypes


class PathPlanningClient
{
public:
    PathPlanningClient();
    ~PathPlanningClient();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    // * return status as actionResultTypeEnum
    actionResultTypeEnum iterate();

    // legacy spinner, which was used when PathPlanning had its own process,
    // before being integrated into motionPlanning as library
    void spin();

    // quick&dirty intercept experiment
    // if this works and if this direction the way to go (alternative to tricky tuning?),
    // then generalize towards a (re)setConfig interface, e.g. setConfig("limits.normal.maxVelRz", 0.1);
    void setRzLimitsOverride(float maxVelRz, float maxAccRz)
    {
        _overrideRzLimits = true;
        _overrideRzMaxVel = maxVelRz;
        _overrideRzMaxAcc = maxAccRz;
    }
    void unsetRzLimitsOverride()
    {
        _overrideRzLimits = false;
    }
private:
    bool _overrideRzLimits = false;
    float _overrideRzMaxVel = 0.0;
    float _overrideRzMaxAcc = 0.0;
};

#endif

