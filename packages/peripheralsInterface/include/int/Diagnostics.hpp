 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * Diagnostics.hpp
 *
 *  Created on: Dec 1, 2016
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_DIAGNOSTICS_HPP_
#define INCLUDE_INT_DIAGNOSTICS_HPP_

#include <thread>

#include <cDiagnostics.hpp>
#include <rosMsgs/t_diag_halmw.h>

#include "ext/peripheralInterfaceNames.hpp"

#include "int/PeripheralsInterfaceData.hpp"
#include "int/VoltageMonitor.hpp"

class Diagnostics {
public:
	Diagnostics(PeripheralsInterfaceData& piData, bool ballhandlersAvailable);

	void start();
	void stop();

private:
	PeripheralsInterfaceData& piData;
	VoltageMonitor voltageMonitor;
	diagnostics::cDiagnosticsSender<rosMsgs::t_diag_halmw> diagSender; // only diagnostics thread

	std::thread diagnosticsThread;
	bool started;

	size_t previousNumberOfDevices;
	size_t desiredNumberOfDevices;

	size_t numberOfOnlineBoards;
	bool previousLeftMotionBoardOnline;
	bool previousRightMotionBoardOnline;
	bool previousRearMotionBoardOnline;
	bool previousLeftBallhandlerBoard;
	bool previousRightBallhandlerBoard;

	void timer();
	void addSensorData();
	void addStatus();
	void addBoardStatus();

	void logDeviceStatus(size_t numberOfDevices);
	void logBoardStatus(bool previousBoardOnline, bool boardOnline, string name);

	bool updateBoardStatus(bool boardOnline);
};

#endif /* INCLUDE_INT_DIAGNOSTICS_HPP_ */
