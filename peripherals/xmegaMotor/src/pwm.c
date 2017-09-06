// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "communication.h"
#include "global.h"
#include "pid.h"
#include "pwm.h"
#include "tc.h"

static int16_t pwmValue = 0; // current pwm value
static int16_t pwmValuePrevious = 0; // previous pwm value
static uint16_t pwmLimit = 0; // set from PC to limit the power to the motor
static int16_t pwmDelta = 0; // set from PC to maximal power increase (PWM) per 2.5ms, use signed to reduce the casting
static uint16_t error = PWM_ERROR_INIT_NOT_PERFORMED;

void initPwm() {
	error = 0;
	pwmValue = 0;
	pwmValuePrevious = 0;
	pwmLimit = 0;
	pwmDelta = 20;

	// setup pwm timer TCD0 for pwm a and pwm b (checkout AVR1306 Xmega Timer/Counter)
	// from http://asf.atmel.com/docs/3.0.1/xmega.services.pwm.example2.xmega-a1_xplained/html/xmega_tc_quickstart.html#xmega_tc_qs_pwm
	tc_enable(&TCD0); // enable timer/counter TCD0
	tc_set_wgm(&TCD0, TC_WG_SS); // set	timer/counter in single slope PWM mode.

	// system clock 32MHz / 512 = 62.5kHz = 16us
	// use 510 below, so 511 (9 bits) gives 100% (no spikes)
	// tc_write_period(&TCD0, 510); // measurement shows 15.7us = 63.7kHz
	// 32 MHz / 50kHz = 640
	// measurement shows 648 counts results in 20us = 50kHz
#define PWM_COUNTS 648
	tc_write_period(&TCD0, PWM_COUNTS);

	// The HiRes extension increases the PWM output resolution by a factor 4 (checkout AVR1311 Xmega Timer/Counter Extensions)
	// The Timer/Counter itself is not running at 4 times the system clock frequency, but it will count by 4 for each system clock cycle. In other words, the 2 LSBs are always 0.
	// So this will not bring any accuracy
	// #define SUPERHIGHRES	0x04
	// tc_hires_set_mode(&HIRESD, HIRES_HREN_TC0_gc | SUPERHIGHRES);
	tc_write_cc(&TCD0, TC_CCC, 0); // set compare match value (0% duty cycle) on CCC = PWM_A
	// Note: if the counters are the same communication errors appear ?EMI?, a difference of +1 or -1 will solve the issue
	tc_write_cc(&TCD0, TC_CCD, 0); // set compare match value (0% duty cycle) on CCD = PWM_B
	tc_enable_cc_channels(&TCD0, TC_CCCEN); // enable compare channel C to pin PD2 = pin 28 = PWM_A
	tc_enable_cc_channels(&TCD0, TC_CCDEN); // enable compare channel D to pin PD3 = pin 29 = PWM_B
	tc_write_clock_source(&TCD0, TC_CLKSEL_DIV1_gc); // set the clock source
}

void taskPwm() {
	// calculate new pwm value from pid and send this value to the motor
	// the value pwm functional range is from -648 to +648, which can be achieved by a pid value of -648*256 to +648*256
	// -648 and +648 result in 100% PWM

	// for testing the setpoint can be directly set to the pwm (without pid)
	if( getMode() == MODE_PWM_ONLY ) {
		pwmValue = getPrimarySetPoint(); // only used for testing
	} else {
		pwmValue = getPidPrimaryResult16(); // default use the pid value
	}

	// limit the pwm within the available range (time)
	if( pwmValue > PWM_COUNTS ) {
		pwmValue = PWM_COUNTS;
	} else if( pwmValue < -PWM_COUNTS ) {
		pwmValue = -PWM_COUNTS;
	}

	// limit the maximal amount of power to the motor (set from PC)
	int16_t pwmLimitNeg = - (int16_t)pwmLimit; // force the type cast
	if( pwmValue > (int16_t)pwmLimit ) {
		error |= PWM_ERROR_POSITIVE_LIMIT;
		pwmValue = pwmLimit;
	} else  if( pwmValue < pwmLimitNeg ) {
		error |= PWM_ERROR_NEGATIVE_LIMIT;
		pwmValue = pwmLimitNeg;
	}

	// limit the maximal increase for the pwm per 2.5ms
	if( pwmValue > 0 ) {
		int16_t pwmValuePosMax = pwmValuePrevious + pwmDelta;
		if( pwmValue > pwmValuePosMax ) {
			error |= PWM_ERROR_POSITIVE_DELTA;
			pwmValue = pwmValuePosMax;
		}
	} else if( pwmValue < 0 ) {
		int16_t pwmValueNegMax = pwmValuePrevious - pwmDelta;
		if( pwmValue < pwmValueNegMax ) {
			error |= PWM_ERROR_NEGATIVE_DELTA;
			pwmValue = pwmValueNegMax;
		}
	}
	pwmValuePrevious = pwmValue; // store current pwm value for the next cycle

	uint16_t timerPWM_A = 0; // default disable, overwrite when motor enabled and valid value
	uint16_t timerPWM_B = 0;
	// when new packets not received in time safety will set mode to MODE_COMMUNICATION_TIMEOUT
	if( getMode() == MODE_COMMUNICATION_TIMEOUT || getMode() == MODE_DISABLE_MOTOR_POWER || getMode() == MODE_ZERO_CURRENT_CALIBRATION ) {
		timerPWM_A = 0; // default already 0, but make explicit with this statement
		timerPWM_B = 0;
	} else {
		if( pwmValue > 0 ) {
			timerPWM_A = pwmValue;
			timerPWM_B = 0;
		} else if( pwmValue < 0 ) {
			timerPWM_A = 0;
			timerPWM_B = -pwmValue;
		}
	}

	// If the timer is near the end of the range e.g. 645, then the jitter will cause that the FET
	// off time is to small, causing over current errors.
	// To prevent this from happening all values near the end will be handled as 100% on (no PWM)
	if( timerPWM_A > (PWM_COUNTS-10) ) { timerPWM_A = PWM_COUNTS + 10; }
	if( timerPWM_B > (PWM_COUNTS-10) ) { timerPWM_B = PWM_COUNTS + 10; }

	tc_write_cc_buffer(&TCD0, TC_CCC, timerPWM_A ); // set compare match value on CCC = PWM_A
	tc_write_cc_buffer(&TCD0, TC_CCD, timerPWM_B ); // set compare match value on CCD = PWM_B
}

int16_t getPwmValue() {
	return pwmValue;
}

void setPwmLimit(uint16_t value) {
	pwmLimit = value;
}

uint16_t getPwmLimit() {
	return pwmLimit;
}

void setPwmDelta(uint16_t value) {
	pwmDelta = (int16_t)value;
}

uint16_t getPwmDelta() {
	return (uint16_t)pwmDelta;
}

uint16_t getPwmError() {
	return error;
}

void clearPwmError(uint16_t value) {
	if( ( value & PWM_ERROR_REQUEST_CLEAR_STATE ) != 0 ) { // for a full clear, also clear the state of pwm
		pwmValue = 0;
	}
	error &= ~value; // clear only the bits set in value
}

