 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 
#ifndef CPERIPHERALSINTERFACETYPES_HPP
#define CPERIPHERALSINTERFACETYPES_HPP

#include <functional>

#include "FalconsCommon.h"
#include "pose2d.hpp"

#include <boost/numeric/ublas/matrix.hpp>
#include <boost/numeric/ublas/lu.hpp>

struct piVelAcc {
	float m1_vel;
	float m2_vel;
	float m3_vel;
	float m1_acc;
	float m2_acc;
	float m3_acc;
};

struct passBall {
	bool activate;
	float bhLeftSpeed;
	float bhRightSpeed;
};

struct piDisplacement {
	float m1_pos;
	float m2_pos;
	float m3_pos;
};

struct pidSettings {
	float p;
	float i;
	float d;
	float iTh;
	int iMax;
};

struct BallhandlerBoardSettings {
	bool ballhandlerPlotEnabled;
	pidSettings pid;
	pidSettings anglePid;
	size_t maxPwmValue;
	size_t maxPwmStepValue;
};

struct MotionBoardSettings {
	bool motorPlotEnabled;
	pidSettings pid;
	size_t maxPwmValue;
	size_t maxPwmStepValue;
};

struct MotionSettings {
	boost::numeric::ublas::matrix<double> matrix;
	boost::numeric::ublas::matrix<double> inverseMatrix;
};

enum BallhandlerBoardControlMode {
	BALLHANDLER_CONTROL_MODE_OFF,
	BALLHANDLER_CONTROL_MODE_ON
};

struct BallhandlerSettings {
	BallhandlerBoardControlMode controlMode;
};

struct BallhandlerBoardSetpoints {
	float setPointAngle;
	float velocity;
};

struct MotorControllerBoardData {
	bool ledYellow;
	bool ledGreen;
	float pwm;
	float currentChannelA;
	float currentChannelB;
	float measureTime;
	float calculationTime;
	float voltage;
	float boardTemperature;
	float motorTimeout;
	int mode;
	float controllerGain;
	float setpoint;
	float error; //error = setpoint - measured_value
	float integral;//integral = integral + error*dt
	float pidOutput;// output = Kp*error + Ki*integral + Kd*derivative
	pidSettings pid;
	size_t maxPwmValue;
	size_t maxPwmStepValue;
};

struct MotionBoardData {
	double velocity;
	double velocityError;
	double displacementEncTicks;
	double displacementDistance;
	double measuredValue;
	float motorTemperature;
	float pidOutput; // pidOutput from motorControllerBoardData
	float integral; // integral from motorControllerBoardData
	float error; // proportional error from motorControllerBoardData
	float derivative; // proportional error from motorControllerBoardData
};

struct MotionBoardDataOutput {
	MotorControllerBoardData motorController;
	MotionBoardData motion;
};

struct BallhandlerBoardData {
	int tachoZero;
	pidSettings anglePidProperties;
	int tacho;
	int angle;
};

struct BallhandlerBoardDataOutput {
	MotorControllerBoardData motorController;
	BallhandlerBoardData ballhandler;
};

struct BallhandlerSetpoints {
    int angleLeft;
    int angleRight;
    float velocityLeft;
    float velocityRight;
};

struct BallhandlerFeedback {
    int angleLeft;
    int angleRight;
    float velocityLeft;
    float velocityRight;
};

// Update function for PeripheralsInterfaceMotion
typedef std::function<void()> voidFunctionType;

#endif /* CPERIPHERALSINTERFACETYPES_HPP */
