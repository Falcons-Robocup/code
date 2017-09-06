 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cWorldModelAdapter.cpp
 *
 *  Created on: Jun 21, 2015
 *      Author: Tim Kouters
 */

#include "int/cWorldModelAdapter.hpp"

#include "FalconsCommon.h"
#include "WorldModelNames.h"

using std::runtime_error;

cWorldModelAdapter::cWorldModelAdapter()
{
    _robotID = getRobotNumber();
    _lastPosition = Position2D();
    _robotIsActive = false;

    _subWmInfo = _hROS.subscribe(WorldModelInterface::t_wmInfo, 1, &cWorldModelAdapter::worldModel_cb, this);
}

cWorldModelAdapter::~cWorldModelAdapter()
{
    ;
}

void cWorldModelAdapter::worldModel_cb(const worldModel::t_wmInfo::ConstPtr& msg)
{
	_lastPosition.x = msg->locationX;
	_lastPosition.y = msg->locationY;
	_lastPosition.phi = msg->locationTheta;

	_robotIsActive = false;
	if(msg->robotStatus.status_type == rosMsgs::s_robot_status::TYPE_INPLAY)
	{
		_robotIsActive = true;
	}
}

void cWorldModelAdapter::getPosition(Position2D &position)
{
	position = _lastPosition;
}

bool cWorldModelAdapter::isRobotActive()
{
    return _robotIsActive;
}
