 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cSetSpeedAdapter.cpp
 *
 *  Created on: Feb 12, 2015
 *      Author: Tim Kouters
 */

#include "int/adapters/cSetSpeedAdapter.hpp"
#include "rosMsgs/t_robotspeed.h"
#include "peripheralsInterface/s_peripheralsInterface_setRobotSpeed.h"
#include "falconsMsgsNames.h"


cSetSpeedAdapter::cSetSpeedAdapter()
{
    // Empty implementation. Defined to allow this class to be globally defined.
	_ppData = NULL;
}

cSetSpeedAdapter::cSetSpeedAdapter(cPathPlanningData &data, iterateFunctionType func)
{
    TRACE(">");
    _ppData = &data;
    _iterateFunc = func;
    TRACE("<");
}

cSetSpeedAdapter::~cSetSpeedAdapter()
{

}

void cSetSpeedAdapter::initializeSetSpeed()
{
    TRACE(">");
    _hROS.reset(new ros::NodeHandle());
    //ros::service::waitForService("s_peripheralsInterface_setRobotSpeed");
    //_pRobotspeed = _hROS->serviceClient<peripheralsInterface::s_peripheralsInterface_setRobotSpeed>("s_peripheralsInterface_setRobotSpeed", USE_SERVICE_PERSISTENCY);
    _pRobotspeed = _hROS->advertise<rosMsgs::t_robotspeed>("g_robotspeed", 5, false);
    TRACE("<");
}

void cSetSpeedAdapter::setSpeed(Velocity2D robotVelocity)
{
    TRACE(">");

    rosMsgs::t_robotspeed msgSpeed;
    msgSpeed.vx = robotVelocity.x;
    msgSpeed.vy = robotVelocity.y;
    msgSpeed.vphi = robotVelocity.phi;
    
    pp_limiters_struct_t limits;
    _ppData->getLimits(limits);

    /* Multiply relative robot speed with speed factor */
    msgSpeed.vx *= limits.relativeSpeedFactorX;
    msgSpeed.vy *= limits.relativeSpeedFactorY;
    msgSpeed.vphi *= limits.relativeSpeedFactorPhi;

    _pRobotspeed.publish(msgSpeed);
    /*
    if (!_pRobotspeed.call(msgSpeed))
    {
        throw std::runtime_error("Failed to get active robots");
    }
    */

    TRACE("published velocity: %2.4f, %2.4f, %2.4f", msgSpeed.vx, msgSpeed.vy, msgSpeed.vphi);

    TRACE("<");
}


