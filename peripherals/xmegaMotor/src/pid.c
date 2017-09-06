// Copyright 2015 Erik Kouters & Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include <compiler.h>
#include "communication.h"
#include "encoder.h"
#include "pid.h"
#include "pwm.h"
#include "safety.h"
#include "xmega.h"

static uint16_t error = PID_ERROR_INIT_NOT_PERFORMED;
static uint8_t mode = 0;
static pidExportT primaryExport = {0,};
static _Bool primaryExportLock = false;

void initPid(pidStateT *PID) {
	error = 0; // clear all pid errors
	primaryExportLock = false;

	mode = MODE_DISABLE_MOTOR_POWER; // PC always has to set mode before the board will provide power to the motor

	PID->integral = 0;
	PID->direction = false;
	PID->derivative = 0;
	PID->pidProperties.p = 0; // the angle pid values shall be set from linux
	PID->pidProperties.i = 0;
	PID->pidProperties.d = 0;
	PID->pidProperties.iTh = 0;
	PID->previousError = 0;
	PID->result16 = 0;
	PID->result32 = 0;
	PID->setPoint = 0; // used as setPoint for the wheel motor or tacho, or as offset in angle mode
}


inline int16_t signedDivideBy65536( int32_t input ) {
	// divide by 65536 including rounding, by using and offset and a shift
	// return input/65536; // is very inefficient
	// a divide of 256 or 65536 takes about 640 cpu cycles cycles = 20us
	if( input < 0 ) {
		return ( input + 32767 ) >> 16; // the 32767 = -0.5 for correct rounding
	} else {
		return ( input + 32768 ) >> 16; // the 32768 = +0.5 for correct rounding
	}
}

inline void calcPid( pidStateT *var, int16_t feedback, int16_t offset ) {
	// The velocity is measured for a fixed period of 2ms.
	// So deltaTime is 2ms.
	// In this period the:
	//  - wheel motor encoder velocity can not exceed the -1000 to 1000 range
	//  - ball handler motor tacho can not exceed the 671 (0.28v) (max left rpm) to 3983 (2.31v) (max right rpm) range (no load @ 26V battery)
	//  - ball handler angle can not exceed the 169 (0.00v) (max left rpm) to 4095 (2.38v)

	// previousError = 0
	// integral = 0
	//start:
	//  error = setpoint - currentVelocity
	//  integral = integral + error*deltaTime
	//  derivative = (error - previous_error)/deltaTime
	//  output = Kp*error + Ki*integral + Kd*derivative
	//  previousError = error
	//  wait(deltaTime)
	//  goto start

	// By applying the scale factors in the Ki and Kd settings we move the scaling from
	// the Xmega to linux, which reduces Xmega cpu cycles and reduces rounding errors

	// Compute velocity error (currentError) from current velocity (feedback).
	// When using in the angle mode, the offset is used to change the setPoint
	// wheel motor velocity in range -1000 to 1000, maximal error (2 inputs) will be then in range from -2000 to 2000
	// ball handler motor tacho in range 500 to 4000, maximal error (3 inputs) error will be then in range from -3500-4000 to 3500+4000
	// ball handler angle range is from 169 to 4095, maximal error (2 inputs) will be then in range from -4000 to 4000
	// so worst case error range is -7500 to 7500 which does not generate an overflow in int16_t
	int16_t currentError = var->setPoint - feedback + offset;

	// Set PID error to zero it is under the threshold of iTh
	if(abs(currentError) < var->pidProperties.iTh)
	{
		currentError = 0;
	}

	// integral = integral + error * deltaTime
	// integral range can be large, but will probably average out
	var->integral += currentError; // currentError in range -7500 to 7500 which fits a large amount of times in int32_t

	if( var->pidProperties.i == 0 || getMode() == MODE_PWM_ONLY ) {
		// when integral is not used in the feedback it can keep on growing
		var->integral = 0;
	}

	// int16_t derivative = (error - prevError) / deltaTime
	// maximal derivative (2 x maximal error) = -15000 to 15000 which does not generate an overflow in int16_t
	var->derivative = currentError - var->previousError;

	// Compute result (example values shown below for wheel motor)
	// result = (pidProperties.p * error) + (pidProperties.i * integral) + (pidProperties.d * derivative);
	// currentError maximal contribution (65535<<4)*2000 -> configure with relative high p value
	// integral maximal contribution 65535*largeNumber -> configure with relative low i value
	// derivative maximal contribution (65535<<4)*4000 -> configure with relative high d value (the derivative is only a peak!)
	// The scaling of the 3 components is part of the pid values set from linux
	// the int32_t casting is required otherwise the compiler "optimizes" to ?smaller? types

	// by using a large divide on the pid result we can use integers instead of fractions for the kp, ki and kd
	// lets say we want to drive full speed and only want to use kp. Full speed setPoint is 1000
	// pwm = 100% => pwm = 648 => pid result = 648*65536 = 42467328
	// error * kp = pid result => max kp = 65536 => error = 42467328/65536 = 648
	// that means the real velocity = setPoint - error = 1000 - 648 = 352 which is way to slow
	// probably the setPoint / error relation should be less then 10%
	// setPoint = 1000 then error less then 100 (and velocity above 900)
	// kp = 4246732/100 = 424673 => which requires 19 bits instead of 16 bits, so a shift of 4 up results in 20 bits which gives enough range and granularity
	// e.g. kp = 17000, pwm 100% => pwm = 648 => pid result = 648*65536 = 42467328
	// error = 42467328/(17000<<4) = 156
	// velocity = setPoint - error = 1000 - 156 = 844

	// both shifts of 4 take about 50 clock cycles, can be optimized to shift ki and change shift of result
	int32_t kp = ((int32_t)var->pidProperties.p) << 4; // scale proportional to be able to achieve the full 100% pwm power
	int32_t ki = ((int32_t)var->pidProperties.i);
	int32_t kd = ((int32_t)var->pidProperties.d) << 4;
	var->result32 = kp * currentError + ki * var->integral + kd * var->derivative;

	var->result16 = signedDivideBy65536( var->result32 );

	// store error for next cycle
	var->previousError = currentError;
}

void taskPid() {
	// Use the latest set point(s) and latest encoder/tacho velocity and or ball handler to calculate the new motor power.

	// depending on the mode use one or two pid loops and or use feedback from the encoder or tacho
	int16_t velocity = getEncoderData().velocity;
	if( mode == MODE_PID_ENCODER ) {
		calcPid( &primaryPID, velocity, 0 ); // keep rpm stable by using encoder data
	} else if( mode == MODE_PID_TACHO ) {
		calcPid( &primaryPID, getSafety().tacho, 0 ); // keep rpm stable by using tacho
	} else if( mode == MODE_PID_ANGLE ) {
		// The primary pid loop uses the the primaryPID.setPoint to keep the tacho stable (motor same rpm)
		// However, depending on to the ball handler angle an offset shall be applied to the motor rpm
		// Calculate the offset from the angle of the ball handler (angle determined through the hall sensor)
		calcPid( &anglePID, getSafety().angle, 0 );
		// Use the calculated offset to change the tacho
		int16_t tachoOffset = anglePID.result16; // keep rpm stable by using tacho, but change for different ball handler angle
		if( anglePID.direction ) {
			tachoOffset = -tachoOffset; // invert the angle relation to tacho (e.g. ball handler up decrease tacho clockwise or anti clockwise, depending on left or right ball handler)
		}
		calcPid( &primaryPID, getSafety().tacho, tachoOffset );
	}

	// prepare values for export a consistent set (atomic)
	if( ! primaryExportLock ) {
		// only copy when not read by communication, otherwise skip
		primaryExport.velocity = velocity;
		primaryExport.pidError = primaryPID.previousError;
		primaryExport.integral = primaryPID.integral;
		primaryExport.result32 = primaryPID.result32;
	}

	// ball handler angle pid range checking (the ranges are described in the picCalc function)
	if( mode == MODE_PID_ANGLE ) {
		if( anglePID.previousError < -4000 || anglePID.previousError > 4000 ) { error |= PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE; }
		// TODO: update with the expected integral range
	  	if( anglePID.integral < -100000 ) {
	  		anglePID.integral = -100000;
	  		error |= PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE;
	  	} else if( anglePID.integral > 100000 ) {
	  		anglePID.integral = 100000;
	  		error |= PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE;
	  	}
		if( anglePID.derivative < -8000 || anglePID.derivative > 8000 ) { error |= PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE; }
		// tacho delta less then 2000 (tachoZero - 2000 ~= 2330 - 2000 = 300 to tachoZero + 2000 ~= 2330 + 2000 = 4330)
		if( anglePID.result16 < -2000 || anglePID.result16 > 2000 ) { error |= PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE; }
	}

	// encoder or tacho pid range checking (the ranges are described in the picCalc function)
	if( mode == MODE_PID_ENCODER ) {
		if( primaryPID.previousError < -2000 || primaryPID.previousError > 2000 ) { error |= PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE; }
		// TODO: update with the expected integral range
	   	if( primaryPID.integral < -100000 ) {
	   		primaryPID.integral = -100000;
	   		error |= PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE;
	   	} else if( primaryPID.integral > 100000 ) {
	   		primaryPID.integral = 100000;
	   		error |= PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE;
	   	}
		if( primaryPID.derivative < -4000 || primaryPID.derivative > 4000 ) { error |= PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE; }
	} else if( mode == MODE_PID_TACHO || mode == MODE_PID_ANGLE ) {
		if( primaryPID.previousError < -7500 || primaryPID.previousError > 7500 ) { error |= PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE; }
		// TODO: update with the expected integral range
	   	if( primaryPID.integral < -100000 ) {
	   		primaryPID.integral = -100000;
	   		error |= PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE;
	   	} else if( primaryPID.integral > 100000 ) {
	   		primaryPID.integral = 100000;
	   		error |= PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE;
	   	}
		if( primaryPID.derivative < -15000 || primaryPID.derivative > 15000 ) { error |= PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE; }
	}
	// pwm range -648 to 648
	if( primaryPID.result16 < -1300 || primaryPID.result16 > 1300 ) { error |= PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE; }

}

pidExportT getPidPrimaryExport( ) {
	pidExportT retVal;
	primaryExportLock = true;
	retVal = primaryExport;
	primaryExportLock = false;
	return retVal;

}


void setAngleSetPoint( int16_t value ) {
	anglePID.setPoint = value;
}

void setPrimarySetPoint( int16_t value ) {
	primaryPID.setPoint = value;
}

void setAngleDirection( bool value ) {
	anglePID.direction = value;
}

int16_t getAngleSetPoint(){
	return anglePID.setPoint;
}

int16_t getPrimarySetPoint(){
	return primaryPID.setPoint;
}

bool getAngleDirection() {
	return anglePID.direction;
}

void setMode(uint8_t value) {
	mode = value;
	// to prevent startup glitches, clear the setPoint and state of the PID when changing mode
	if( mode == MODE_PID_TACHO ) {
		anglePID.setPoint = 0; // angle not use in this mode
		primaryPID.setPoint = getAngleTachoZero().tacho; // default setPoint for tacho is not 0
	} else if( mode == MODE_PID_ANGLE ) {
		anglePID.setPoint = getAngleTachoZero().angle; // default setPoint for angle is not 0
		primaryPID.setPoint = getAngleTachoZero().tacho; // default setPoint for tacho is not 0
	} else {
		anglePID.setPoint = 0;
		primaryPID.setPoint = 0;
	}

	anglePID.result16 = 0;
	anglePID.result32 = 0;
	anglePID.previousError = 0;
	anglePID.integral = 0;
	primaryPID.result16 = 0;
	primaryPID.result32 = 0;
	primaryPID.previousError = 0;
	primaryPID.integral = 0;
}

uint8_t getMode(){
	return mode;
}

void setPidAngleProperties(pidPropT value) {
	anglePID.pidProperties = value;
}

void setPidPrimaryProperties(pidPropT value) {
	primaryPID.pidProperties = value;
}

pidPropT getPidAngleProperties() {
	return anglePID.pidProperties;
}

pidPropT getPidPrimaryProperties() {
	return primaryPID.pidProperties;
}

int16_t getPidAngleError(){
	return anglePID.previousError;
}

int16_t getPidPrimaryError(){
	return primaryPID.previousError;
}

int32_t getPidAngleIntegral(){
	return anglePID.integral;
}

int32_t getPidPrimaryIntegral(){
	return primaryPID.integral;
}

int16_t getPidPrimaryResult16(){
	return primaryPID.result16;
}

// only used for diagnostics (send to linux pc)
int32_t getPidPrimaryResult32(){
	return primaryPID.result32;
}

uint16_t getPidError() {
	return error;
}

void clearPidError(uint16_t value) {
	if( ( value & PID_ERROR_REQUEST_CLEAR_STATE ) != 0 ) { // for a full clear, also clear the state of pid
		anglePID.derivative = 0;
		anglePID.integral = 0;
		anglePID.previousError = 0;
		anglePID.result16 = 0;
		anglePID.result32 = 0;
		anglePID.setPoint = 0;
		primaryPID.derivative = 0;
		primaryPID.integral = 0;
		primaryPID.previousError = 0;
		primaryPID.result16 = 0;
		primaryPID.result32 = 0;
		primaryPID.setPoint = 0;
	}
	error &= ~value; // clear only the bits set in value
}

