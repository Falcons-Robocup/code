 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cBallHandlerAdapter.cpp
 *
 *  Created on: May 21, 2017
 *      Author: Raf van Son
 */

#include <stdexcept>
#include <cDiagnosticsEvents.hpp>

#include "int/cBallHandlerAdapter.hpp"
#include <WorldModelNames.h>
#include <peripheralInterfaceNames.hpp>
#include "FalconsCommon.h"

using std::string;
using std::exception;
using std::runtime_error;

cBallHandlerAdapter::cBallHandlerAdapter()
{
	string enableBallHandlers_serviceName = peripheralInterfaceServiceNames::enableBallHandlers;
	string disableBallHandlers_serviceName = peripheralInterfaceServiceNames::disableBallHandlers;

	ros::service::waitForService(enableBallHandlers_serviceName);
	ros::service::waitForService(disableBallHandlers_serviceName);

	_cEnableBH = _hROS.serviceClient<peripheralsInterface::s_enable_ballHandlers>(enableBallHandlers_serviceName, USE_SERVICE_PERSISTENCY);
	_cDisableBH = _hROS.serviceClient<peripheralsInterface::s_disable_ballHandlers>(disableBallHandlers_serviceName, USE_SERVICE_PERSISTENCY);
}

cBallHandlerAdapter::~cBallHandlerAdapter()
{

}

void cBallHandlerAdapter::enableBH(bool &successful)
{
	peripheralsInterface::s_enable_ballHandlers srvEnableBH;
    TRACE("Enabling BH");
	try
	{
        if(!_cEnableBH.call(srvEnableBH))
        {
        	TRACE_ERROR("Failed to enable BH");
            successful = false;
        }
	}
	catch (exception &e)
	{
		throw e;
	}
}

void cBallHandlerAdapter::disableBH(bool &successful)
{
	peripheralsInterface::s_disable_ballHandlers srvDisableBH;
    TRACE("Disabling BH");
	try
	{
        if(!_cDisableBH.call(srvDisableBH))
        {
        	TRACE_ERROR("Failed to disable BH");
            successful = false;
        }
	}
	catch (exception &e)
	{
		throw e;
	}
}
