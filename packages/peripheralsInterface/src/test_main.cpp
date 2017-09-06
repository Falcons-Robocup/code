 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * test_main.cpp
 *
 *  Created on: Sep 20, 2016
 *      Author: Edwin Schreuder
 */

#include <int/Ballhandlers.hpp>
#include <int/Motion.hpp>
#include <chrono>
#include <thread>

#include "int/PeripheralsInterfaceData.hpp"
#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/motors/MotionBoard.hpp"

#include "int/DeviceManager.hpp"

using namespace std;

int main(int argc, char **argv) {

	PeripheralsInterfaceData _piData;
	DeviceManager deviceManager(_piData);

	deviceManager.start();

	MotionBoard rearMotionBoard = MotionBoard(MOTION_BOARD_REAR, deviceManager);

	float setpoint = 100;
	if (argc > 1) {
		setpoint = atof(argv[1]);
	}

	while(true) {
		this_thread::sleep_for(chrono::milliseconds(100));

		if (rearMotionBoard.isConnected()) {
			rearMotionBoard.getBoardData();
			rearMotionBoard.setSetpoint(setpoint);
		}
		else {
			// Rear MotionController is not available
//			cout << "Rear Motion Board not available: " << endl;
		}
	}
}
