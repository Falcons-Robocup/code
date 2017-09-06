// Copyright 2015 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include "xmega.h"
#include "encoder.h"
#include "tc.h"
#include "qdec.h"

static uint16_t error = PID_ERROR_INIT_NOT_PERFORMED;
static encDataT encData;
static int16_t encoderTicksPrevious = 0;

/* Brief setup the counter used to decode and count the encoder pulses (from the wheel or ball handler)
 * The encoder uses the Xmega Quadrature Decoder to create the encoder count
 *
 * The Maxon motor encoder is providing 3 signal
 *  - enc a = zero degrees
 *  - enc b = 90 degrees
 *  - enc i = index (which is not used for our usage)
 *
 * The motor will give 512 toggles per input for one revolution
 * The 2 pins are 90 degrees shifted so this will result in 2048 tick per revolution
 * The gear box has a ratio of 12:1 resulting in 2048*12 = 24576 ticks per "wheel" revolution
 *
 * The encoder input toggles are not handled by the processor but by a hardware timer
 * because encoder inputs are connected to PORTE, TCE0 or TCE1 has to be used
 * TCE0 is timer/counter 0 on PORTE
 * TCE1 is timer/counter 1 on PORTE
 * For the encoder we use TCE1
 *
 * A filter setting can be configured during phase pin configuration
 * it is expected this will handle the bouncing
 *
 * checkout
 * xdk-asf-3.25.0/xmega/drivers/qdec/example2/qdec_example2.c
 */

static qdec_config_t config;

void initEncoder() {
	error = 0;
	// from xdk-asf-3.25.0/xmega/drivers/qdec/example2/qdec_example2.c
	qdec_get_config_defaults(&config);
	qdec_config_phase_pins(&config, &PORTE, 0, false, 500); // TODO: checkout the filter_us value
	qdec_config_tc(&config, &TCE1);
	// the revolution calculation is performed on a higher level, keep the maximal counter resolution
	// using 65535 will not result in the full 16 bit range (only 0 to 65534)
	// 65536 is the same as 0 and testing shows it results in the full 16 bit range of 0 to 65535
	qdec_config_revolution(&config, 0);
	qdec_enabled(&config);
	encData.displacement = 0;
	encData.velocity = 0;
}

/* Brief call on a regular base to update the displacement and velocity (of the wheel)
 */
void taskEncoder(bool measure) {
	// value from 0 to 65535
	// Invert the encoder value so the feedback is correct when the black (MOT1) and red (MOT2) wheel motor wires are in the
	// same order as the black and red wires of the power input (and the same for the ball handler motors).
	int16_t encoderTicksNew = -TCE1.CNT; // qdec_get_position(&config);
	// if robot moves 10 meter per second
	// a wheel has an outline of 0.317m
	// that would result in 10/0.317 = 31.5 wheel rotations per second (1892.7 rotations per minute)
	// with 24576 ticks per rotation that is 775268.1 ticks per second
	// with an encoder task period of 2ms / 500Hz that will result in 1550.5 ticks per task
	// so the encoderTicks range will be from -1551 to 1551
	// testing with a free running motor at 26VDC (consuming 68Watt) shows a maximum of 1086 (7m/s)
	int16_t encoderTicks = encoderTicksNew - encoderTicksPrevious;

	encData.displacement += encoderTicks;

	// total period is 2.5ms of which 0.5ms for calculation and 2.0ms for measurement
	// velocity is defined as ticks per 2ms
	// which is wheel (500/24576 = 0.020345) wheel rotations per second
	if( measure ) {
		encData.velocity = encoderTicks;
	}

	encoderTicksPrevious = encoderTicksNew;

	// see comment above for the maximal expected ranges
	if( encoderTicks < -1000 ) {
		error |= ENCODER_ERROR_VELOCITY_MAX_NEGATIVE;
	} else if( encoderTicks > 1000 ) {
		error |= ENCODER_ERROR_VELOCITY_MAX_POSITIVE;
	}
}


encDataT getEncoderData() {
	return encData;
}

uint16_t getEncoderError( ) {
	return error;
}

void clearEncoderError( uint16_t value ){
	error &= ~value; // clear only the bits set in value
}
