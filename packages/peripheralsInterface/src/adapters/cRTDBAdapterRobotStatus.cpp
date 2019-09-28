 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBAdapterRobotStatus.cpp
 *
 *  Created on: Oct 30, 2018
 *      Author: Erik Kouters
 */

#include "int/adapters/cRTDBAdapterRobotStatus.hpp"

#include <iostream>
#include <string>

#include <cDiagnostics.hpp>
#include <FalconsCommon.h>
#include "tracing.hpp"

using std::cerr;
using std::endl;
using std::string;

cRTDBAdapterRobotStatus::cRTDBAdapterRobotStatus() {
}

cRTDBAdapterRobotStatus::~cRTDBAdapterRobotStatus() {
}

void cRTDBAdapterRobotStatus::initialize() {
    TRACE_FUNCTION("");

    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
}

void cRTDBAdapterRobotStatus::setRobotStatus(bool inPlay) {
    TRACE_FUNCTION("");

    T_INPLAY_FEEDBACK robotStatus;

	if (inPlay) {
		robotStatus = robotStatusEnum::INPLAY;
	}
	else {
		robotStatus = robotStatusEnum::OUTOFPLAY;
	}

	_rtdb->put(INPLAY_FEEDBACK, &robotStatus);
}
