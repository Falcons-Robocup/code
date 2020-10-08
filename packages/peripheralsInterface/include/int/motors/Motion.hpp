 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #ifndef MOTION_HPP
#define MOTION_HPP

#include "int/PeripheralsInterfaceTypes.hpp"

#include "int/motors/PeripheralsInterfaceData.hpp"
#include "int/motors/DeviceManager.hpp"
#include "int/motors/MotionBoard.hpp"

using namespace std;

class MotionMotor {
public:
	float setpoint;
	MotionBoardDataOutput data;
	bool online;
	string name;
};

class Motion {

public:
	Motion(PeripheralsInterfaceData &piData, MotionBoard& leftMotionBoard, MotionBoard& rightMotionBoard, MotionBoard& rearMotionBoard);
	~Motion();

	void update();

private:
	void watchDog();

	void updateStatus();
	void updateBoardData();
	void updateBoardSetpoints();

	void calculateEncodersInput();
	void calculateVelocityAndDisplacementOutput();

	void traceData();

	PeripheralsInterfaceData &_piData;

	size_t _watchDogCycles;

	MotionBoard& _leftMotionBoard;
	MotionBoard& _rightMotionBoard;
	MotionBoard& _rearMotionBoard;

	MotionMotor _leftMotor;
	MotionMotor _rightMotor;
	MotionMotor _rearMotor;
};
#endif /* CPERIPHERALSINTERFACEMOTION_HPP */
