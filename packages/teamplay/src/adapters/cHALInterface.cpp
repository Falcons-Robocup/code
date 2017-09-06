 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cHALInterface.cpp
 * ROS interface towards peripheralsInterface, but cPeripheralsInterfaceInterface sounds really awful
 *
 *  Created on: Feb 5, 2017
 *      Author: Coen Tempelaars
 */
#include "FalconsCommon.h"
#include "int/adapters/cHALInterface.hpp"

#include "peripheralInterfaceNames.hpp"
#include "peripheralsInterface/s_disable_ballHandlers.h"
#include "peripheralsInterface/s_enable_ballHandlers.h"

#include "int/utilities/trace.hpp"


cHALInterface::cHALInterface()
   : _ballhandlersAreEnabled(false)
{
}

cHALInterface::~cHALInterface()
{
}

void cHALInterface::connect()
{
    if (!isGoalKeeper())
    {
        n.reset(new ros::NodeHandle());

        ros::service::waitForService(peripheralInterfaceServiceNames::disableBallHandlers);
        _disableBallhandlersService = n->serviceClient<peripheralsInterface::s_disable_ballHandlers>
                                               (peripheralInterfaceServiceNames::disableBallHandlers, true);

        ros::service::waitForService(peripheralInterfaceServiceNames::enableBallHandlers);
        _enableBallhandlersService = n->serviceClient<peripheralsInterface::s_enable_ballHandlers>
                                               (peripheralInterfaceServiceNames::enableBallHandlers, true);
    }
}

void cHALInterface::disableBallhandlers()
{
    if (!isGoalKeeper())
    {
        peripheralsInterface::s_disable_ballHandlers service_params;

        bool call_succeeded = _disableBallhandlersService.call(service_params);

        if (call_succeeded)
        {
            TRACE("ballhandlers are now disabled");
            _ballhandlersAreEnabled = false;
        }
        else
        {
            TRACE_ERROR("ROS call disable ballhandlers failed.");
            connect();
        }
    }
}

void cHALInterface::enableBallhandlers()
{
    if (!isGoalKeeper())
    {
        peripheralsInterface::s_enable_ballHandlers service_params;

        bool call_succeeded = _enableBallhandlersService.call(service_params);

        if (call_succeeded)
        {
            TRACE("ballhandlers are now enabled");
            _ballhandlersAreEnabled = true;
        }
        else
        {
            TRACE_ERROR("ROS call enable ballhandlers failed.");
            connect();
        }
    }
}

bool cHALInterface::areBallhandlersEnabled()
{
    return _ballhandlersAreEnabled;
}
