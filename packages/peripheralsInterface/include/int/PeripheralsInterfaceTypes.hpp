// Copyright 2016-2020 Edwin Schreuder (Falcons)
// SPDX-License-Identifier: Apache-2.0

#ifndef CPERIPHERALSINTERFACETYPES_HPP
#define CPERIPHERALSINTERFACETYPES_HPP

#include <functional>

#include "falconsCommon.hpp"
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
