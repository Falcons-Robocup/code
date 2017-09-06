// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "global.h"
#include "tc.h"
#include "scheduler.h"

#include "encoder.h"
#include "pid.h"
#include "pwm.h"
#include "spiMaster.h"
#include "safety.h"

static uint16_t error = SCHEDULER_ERROR_INIT_NOT_PERFORMED;
static bool measuring = false;
static schedulerTimeT schedulerTime;

static inline void schedulerTaskActive(){
	PORTE.OUTSET = 0x20; // activate applicationId0 pin
}

static inline void schedulerCalculateActive(){
	PORTE.OUTSET = 0x40; // activate applicationId1 pin
}

static inline void schedulerMeasureActive(){
	PORTE.OUTSET = 0x80; // activate applicationId2 pin
}

static inline void schedulerTaskInactive(){
	PORTE.OUTCLR = 0x20; // deactivate applicationId0 pin
}

static inline void schedulerCalculateInactive(){
	PORTE.OUTCLR = 0x40; // deactivate applicationId1 pin
}

static inline void schedulerMeasureInactive(){
	PORTE.OUTCLR = 0x80; // deactivate applicationId2 pin
}

// The scheduler task is called from the timer interrupt.
// The task is toggling between measuring mode and calculation mode.
// The timer is reprogrammed to 2000us for the measuring task and 500us
// for the calculation task
// Total period time : 2500us = 400Hz
// In the calculation mode the velocity error is calculated and used to correct the motor power.
// In the measuring mode the motor velocity is measured.
// During the measurement the motor power (PWM) is not changed
// During the measurement period also the safety check is performed.
// The velocity measurement is not performed during the calculation mode because
// during that mode the motor power could change which introduces a significant velocity error.
// During each task the amount of 32MHz cycles is counted and use to
// check if the task did not was using too much time.
// The scheduler can only be interrupted by the usart and adc, both are very short probably below 100cpu cycles ~ 3us
// The scheduler time / jitter can be measured with an oscilloscope on application ID0, ID1 and ID2
static void taskScheduler(void){ // Interrupt
	// toggle between measuring (determine velocity) and updating (update PWM value)
	if( measuring ) { // do not change PWM state, measure velocity
		uint16_t startTime =TCC0.CNT;
		schedulerMeasureActive(); // application ID 2 high
		schedulerTaskInactive(); // application ID 0 low

		// re-configure timer for measurement mode
		// typically this is the long state e.g. 2ms / 80% of time
		// 32MHz / 2.0ms = 64000
		tc_write_period(&TCF0, 64000);

		// perform the tasks during the measurement cycle
		taskEncoder(false); // update the displacement but discard the velocity because the PWM recently might have changed
		taskSpiMaster(); // get over current and or over current errors from the drv8301 motor driver
		taskSafety(); // get the actual current

		// done: change mode for next cycle and perform some checking
		measuring = false;

		schedulerTime.measure = TCC0.CNT - startTime; // this is always 1717 (pretty stable)
		// processing during measuring should not take more then 5000 cycles = 156.25us (of the available 2000us)
		if( schedulerTime.measure > 5000 ) { error |= SCHEDULER_ERROR_MEASURE_TIME_EXPIRED; }
		schedulerMeasureInactive(); // application ID 2 low
	} else { // if required change PWM state, in this state the velocity might have a significant error
		uint16_t startTime = TCC0.CNT;
		schedulerCalculateActive(); // application ID 1 high
		schedulerTaskActive(); // application ID 0 high

		// re-configure timer for calculation mode
		// typically this is the short state e.g. 0.5ms / 20% of time
		// 32MHz / 0.5ms = 16000
		tc_write_period(&TCF0, 16000);

		// during the previous me
		taskEncoder(true); // the previous 2ms the PWM has been stable, now capture the velocity
		// use the latest velocity, set point and the pid settings to calculate new values for the PWM
		taskPid();
		taskPwm();

		// done: change mode for next cycle and perform some checking
		measuring = true;

		// TODO: find out why there is a difference in CPU cycles for positive and negative PWM (also in MODE_PWM_ONLY)
		schedulerTime.calculation = TCC0.CNT - startTime; // this is between 965 and 973 for positive PWM and 984-992 for negative PWM
		// processing during calculation should not take more then 2000 cycles = 62.5us (of the available 500us)
		if( schedulerTime.calculation > 2000 ) { error |= SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED; }
		schedulerCalculateInactive(); // application ID 1 low

	}
}

// configure the scheduler task timer and duration check timer
void initScheduler() {
	error = 0;
	// detailed information about timer configuration
	// from http://asf.atmel.com/docs/3.0.1/xmegaa/html/xmega_tc_quickstart.html

	// use TCF0 as counter for the scheduler measuring (2000us) and calculation (500us) task
	tc_enable(&TCF0);
	tc_set_overflow_interrupt_callback(&TCF0, taskScheduler);
	tc_set_wgm(&TCF0, TC_WG_NORMAL);
	tc_write_period(&TCF0, 1000); // value is only used to call the taskScheduler once then the new value will be set from this task
	tc_set_overflow_interrupt_level(&TCF0, TC_INT_LVL_LO);
	tc_write_clock_source(&TCF0, TC_CLKSEL_DIV1_gc);

	// Use TCC0 as free running counter to check if measuring task finishes within 2000us
	// and calculation task within 500us.
	tc_enable(&TCC0);
	tc_set_wgm(&TCC0, TC_WG_NORMAL);
	tc_write_clock_source(&TCC0, TC_CLKSEL_DIV1_gc);

	schedulerTime.measure = 0;
	schedulerTime.calculation = 0;
}

uint16_t getSchedulerError() {
	return error;
}

void clearSchedulerError( uint16_t value ){
	error &= ~value; // clear only the bits set in value
}

schedulerTimeT getSchedulerTime( ){
	return schedulerTime;
}
