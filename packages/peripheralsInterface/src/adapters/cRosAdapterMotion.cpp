 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRosAdapterMotion.hpp
 *
 *  Created on: Mar 1, 2016
 *      Author: Tim Kouters
 */

#include "int/adapters/cRosAdapterMotion.hpp"

#include <falconsMsgsNames.h>

#include "ext/peripheralInterfaceNames.hpp"

#include "int/PeripheralsInterfaceData.hpp"

static rosMsgs::t_motor_pid_params_per_motor boardDataToPidParameters(MotionBoardDataOutput &boardDataOutput);

cRosAdapterMotion::cRosAdapterMotion(PeripheralsInterfaceData& piData) :
		_piData(piData) {
	TRACE(">");

	TRACE("<");
}

cRosAdapterMotion::~cRosAdapterMotion() {
	TRACE(">");

	TRACE("<");
}

void cRosAdapterMotion::initialize() {
	TRACE(">");

	_subTarget = _n.subscribe(
			falconsMsgsInterface::g_robotspeed, 20, &cRosAdapterMotion::cbRobotSpeed, this);

	_tMotorPidParameters = _n.advertise<rosMsgs::t_motor_pid_params>(
			peripheralInterfaceTopicNames::topicPIDParameters, 20);
	_tMotorRobotSpeed = _n.advertise<rosMsgs::t_robotspeed>(
			peripheralInterfaceTopicNames::topicMotorRobotSpeed, 20);

	_srvGetMotionPID = _n.advertiseService(
			peripheralInterfaceServiceNames::serviceGetMotionPID, &cRosAdapterMotion::cbGetMotionPID, this);
	_srvSetMotionPID = _n.advertiseService(
			peripheralInterfaceServiceNames::serviceSetMotionPID, &cRosAdapterMotion::cbSetMotionPID, this);
	_srvSetRobotSpeed = _n.advertiseService(
			peripheralInterfaceServiceNames::serviceSetRobotSpeed, &cRosAdapterMotion::cbSetRobotSpeed, this);

	TRACE("<");
}

void cRosAdapterMotion::publishRobotSpeed() {
	rosMsgs::t_robotspeed motorRobotSpeedMsg;

	piVelAcc vel = _piData.getVelocityOutput();
	motorRobotSpeedMsg.vx = vel.vel.x;
	motorRobotSpeedMsg.vy = vel.vel.y;
	motorRobotSpeedMsg.vphi = vel.vel.phi;
	_tMotorRobotSpeed.publish(motorRobotSpeedMsg);
}

void cRosAdapterMotion::publishPidParameters() {
	rosMsgs::t_motor_pid_params motorPidParamsMsg;
	MotionBoardDataOutput boardDataOutput;

	boardDataOutput = _piData.getLeftMotionBoard().getDataOutput();
	motorPidParamsMsg.m1 = boardDataToPidParameters(boardDataOutput);
	motorPidParamsMsg.m1.motorId = 1;

	boardDataOutput = _piData.getRightMotionBoard().getDataOutput();
	motorPidParamsMsg.m2 = boardDataToPidParameters(boardDataOutput);
	motorPidParamsMsg.m2.motorId = 2;

	boardDataOutput = _piData.getRearMotionBoard().getDataOutput();
	motorPidParamsMsg.m3 = boardDataToPidParameters(boardDataOutput);
	motorPidParamsMsg.m3.motorId = 3;

	_tMotorPidParameters.publish(motorPidParamsMsg);
}

bool cRosAdapterMotion::cbGetMotionPID(
		peripheralsInterface::s_peripheralsInterface_getMotionPID::Request& request,
		peripheralsInterface::s_peripheralsInterface_getMotionPID::Response& response) {
	TRACE(">");

	// TODO: To be implemented.

	TRACE("<");

	return true;
}

bool cRosAdapterMotion::cbSetMotionPID(
		peripheralsInterface::s_peripheralsInterface_setMotionPID::Request& request,
		peripheralsInterface::s_peripheralsInterface_setMotionPID::Response& response) {
	TRACE(">");

	pidSettings pid;
	MotionBoardSettings settings;

	pid.p = request.P;
	pid.i = request.I;
	pid.d = request.D;
	pid.iTh = request.iTh;

	settings = _piData.getLeftMotionBoard().getSettings();
	settings.pid = pid;
	_piData.getLeftMotionBoard().setSettings(settings);

	settings = _piData.getRightMotionBoard().getSettings();
	settings.pid = pid;
	_piData.getRightMotionBoard().setSettings(settings);

	settings = _piData.getRearMotionBoard().getSettings();
	settings.pid = pid;
	_piData.getRearMotionBoard().setSettings(settings);

	TRACE("<");

	return true;
}

bool cRosAdapterMotion::cbSetRobotSpeed(
		peripheralsInterface::s_peripheralsInterface_setRobotSpeed::Request& request,
		peripheralsInterface::s_peripheralsInterface_setRobotSpeed::Response& response) {
	TRACE("LATENCY vel.x=%12.9f ; vel.y=%12.9f", request.vx, request.vy);

	piVelAcc vel;
	vel.vel.x = request.vx;
	vel.vel.y = request.vy;
	vel.vel.phi = request.vphi;

	_piData.setVelocityInput(vel);

	return true;
}

void cRosAdapterMotion::cbRobotSpeed(
		const rosMsgs::t_robotspeed::ConstPtr& msg) {
	TRACE("LATENCY vel.x=%12.9f ; vel.y=%12.9f", msg->vx, msg->vy);

	piVelAcc vel;
	vel.vel.x = msg->vx;
	vel.vel.y = msg->vy;
	vel.vel.phi = msg->vphi;

	_piData.setVelocityInput(vel);
}

rosMsgs::t_motor_pid_params_per_motor boardDataToPidParameters(MotionBoardDataOutput &boardDataOutput) {
	rosMsgs::t_motor_pid_params_per_motor motorPidParamsMsg;

	motorPidParamsMsg.ledYellow = boardDataOutput.motorController.ledYellow;
	motorPidParamsMsg.displacementDistance = boardDataOutput.motion.displacementDistance; // in m
	motorPidParamsMsg.displacementEncTicks = boardDataOutput.motion.displacementEncTicks; // in ticks / 2ms
	motorPidParamsMsg.velocity = boardDataOutput.motion.velocity; // in ticks/2.5 ms
	motorPidParamsMsg.velocityError = boardDataOutput.motion.velocityError; // in ticks/ 2.5 m/s
	motorPidParamsMsg.pwm = boardDataOutput.motorController.pwm;
	motorPidParamsMsg.measureTime = boardDataOutput.motorController.measureTime;
	motorPidParamsMsg.calculationTime = boardDataOutput.motorController.calculationTime;
	motorPidParamsMsg.currentChannelA = boardDataOutput.motorController.currentChannelA;
	motorPidParamsMsg.currentChannelB = boardDataOutput.motorController.currentChannelB;
	motorPidParamsMsg.voltage = boardDataOutput.motorController.voltage;
	motorPidParamsMsg.boardTemperature = boardDataOutput.motorController.boardTemperature;
	motorPidParamsMsg.motorTemperature = boardDataOutput.motion.motorTemperature;
	motorPidParamsMsg.systemClock = 0;
	motorPidParamsMsg.motorTimeout = boardDataOutput.motorController.motorTimeout;
	motorPidParamsMsg.controllerGain = boardDataOutput.motorController.controllerGain;

	motorPidParamsMsg.setPoint = boardDataOutput.motorController.setpoint; // in m/s
	motorPidParamsMsg.measuredValue = boardDataOutput.motion.measuredValue;
	motorPidParamsMsg.error = boardDataOutput.motion.errorMs;
	motorPidParamsMsg.integral = boardDataOutput.motion.integralMs;
	motorPidParamsMsg.derivative = boardDataOutput.motion.derivativeMs; // not exported by motor controller? See src/communication.c in xmegamotor.
	motorPidParamsMsg.pidOutput = boardDataOutput.motion.pidOutputMs; // in m/s

	motorPidParamsMsg.P = boardDataOutput.motorController.pid.p;
	motorPidParamsMsg.I = boardDataOutput.motorController.pid.i;
	motorPidParamsMsg.D = boardDataOutput.motorController.pid.d;
	motorPidParamsMsg.iTh = boardDataOutput.motorController.pid.iTh;

	return motorPidParamsMsg;
}
