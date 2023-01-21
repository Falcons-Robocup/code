// Copyright 2015, 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <stdio.h>
#include <string.h> // for memcpy

#include "communication.h"
#include "encoder.h"
#include "pid.h"
#include "pwm.h"
#include "safety.h"
#include "scheduler.h"
#include "spiMaster.h"
#include "xmega.h"
#include "packetSizes.h"

static uint16_t error = 0;
static uint8_t sofAlignmentCounter = 0;

static toBoardPacketT rxPacket = {0,};
static toBoardPacketT rxPacketTmp = {0,};
bool static processRxPacket = false;
static uint8_t lastrxPackedAckId = 0;

static fromBoardPacketT txPacket;

static uint8_t ledGreenBlinkCounter = 0; // incremented slowly for packets from board and fast for packets send to board
static uint8_t ledYellowClearCounter = 0; // used to make single errors visible

// run once during powerup/reset
void initCommunication() {
	rxPacketTmp.sof = SOF; // should never be overwritten
	rxPacketTmp.ackId = 0xff;
	rxPacketTmp.command = 0xff;
	rxPacketTmp.payloadSize = 0;
	rxPacketTmp.checksum = 0xff; // is overwritten when receiving packet, but never read
	rxPacketTmp.endCheck = 0x98; // should never be overwritten

	txPacket.sof = SOF; // should never be overwritten
	txPacket.ackId = 0xff; // incremented with with wrap around on board
	txPacket.feedbackId = 0xff; // return last ackId sender, so sender (PC) can accurate calculate the board worst case RX buffer size
	txPacket.bufferSpace = 0;
	txPacket.responseType = 0;
	txPacket.payloadSize = 0;
	txPacket.checksum = 0;
	txPacket.endCheck = 0x97; // should never be overwritten
}

// Call as fast as possible in spare time (main / low priority).
// When a byte is received from the far end it will add the received byte to the packet.
// If the packet is complete it will process the packet.
// If the packet requests a return value for the far end it will send the requested information.
// If no data request is made by the far end the default data will be send to the far end.
// When an error occurs the error code will be returned to the far end instead of the requested or default data.
void taskCommunication() {

	static uint8_t lastErrorSend = 0; // used to limit the amount of error messages send to the PC when the error value is the same
	static uint16_t communicationErrorPrevious = 0;
	static uint16_t encoderErrorPrevious = 0;
	static uint16_t pidErrorPrevious = 0;
	static uint16_t pwmErrorPrevious = 0;
	static uint16_t safetyErrorPrevious = 0;
	static uint16_t schedulerErrorPrevious = 0;
	static uint16_t spiMasterErrorPrevious = 0;

	// check if one or more byte available in rx buffer, if so process byte(s), else continue
	while( ( ! processRxPacket ) && receiveByteAvailable() ) { receivePacket(); } // if packet complete flag processRxPacket will be set active

	// prepare new packet for far end
	txPacket.sof = SOF;
	txPacket.payloadSize = 0;
	txPacket.responseType = RESP_DEFAULT;

	if( processRxPacket ) {
		ledGreenBlinkCounter+=6; // increase blink speed green led to indicate valid packets have been received
		// start decoding the command from the far end
		processRxPacket = false; // reset flag for the next packet
		rxPacketAckIdCheck();
		if( rxPacketChecksumFail() ) {
			// ignore packet command, communication error flag has been set, next cycle this will be send to the far end
		} else if ( rxPacket.command == CMD_SET_PRIMARY_SETPOINT ) {
			// update current primary setpoint, this command first in the list because command used very often (wheel encoder mode)
			setPrimarySetPoint( rxPacket.pl.s16[0] );
			if( rxPacket.payloadSize != 2 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_SET_ANGLE_SETPOINT ) {
			if( rxPacket.payloadSize != PACKETSIZE_CMD_SET_ANGLE_SETPOINT ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
			// update current angle setpoint, this command first in the list because command used very often (angle mode)
			setAngleSetPoint( rxPacket.pl.s16[0] );
			if( rxPacket.payloadSize != 2 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_LOOPBACK ) {
			// just return the payload
			txPacket.responseType = RESP_LOOPBACK;
			txPacket.payloadSize = rxPacket.payloadSize;
			memcpy((void *) &txPacket.pl.u8[0], (void *) &rxPacket.pl.u8[0], rxPacket.payloadSize ); // TODO: check if this are the correct pointers
		} else if ( rxPacket.command == CMD_AA_GET_BOARD ) {
			// return board properties
			txPacket.responseType = RESP_AA_BOARD;
			txPacket.payloadSize = PACKETSIZE_RESP_AA_BOARD;
			txPacket.pl.u16[0] = DEVICE_ID;
			txPacket.pl.u16[1] = VENDOR_ID;
			txPacket.pl.u16[2] = (uint16_t) getApplicationId(); // TODO: possible to reduce packet with 1 byte
			txPacket.pl.u16[3] = HARDWARE_VERSION;
			txPacket.pl.u16[4] = SOFTWARE_VERSION;
			txPacket.pl.u16[5] = 0; // placeholder;
			txPacket.pl.u32[3] = getXmegaDeviceId(); // caution: payload index
			txPacket.pl.u32[4] = getXmegaSerial();
			txPacket.pl.u32[5] = getCpuClkHz();

		} else if ( rxPacket.command == CMD_GET_LED_GREEN ) {
			txPacket.responseType = RESP_LED_GREEN;
			txPacket.payloadSize = PACKETSIZE_RESP_LED_GREEN;
			if( getLedGreen() ) { txPacket.pl.u8[0] = 1; } else { txPacket.pl.u8[0] = 0; }
		} else if ( rxPacket.command == CMD_SET_LED_GREEN ) {
			// overwrite current led value from the far end
			if( rxPacket.pl.u8[0] == 1 ) { setLedGreen( true ); } else { setLedGreen( false ); }
			if( rxPacket.payloadSize != 1 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_LED_YELLOW ) {
			txPacket.responseType = RESP_LED_YELLOW;
			txPacket.payloadSize = PACKETSIZE_RESP_LED_YELLOW;
			if( getLedYellow() ) { txPacket.pl.u8[0] = 1; } else { txPacket.pl.u8[0] = 0; }
		} else if ( rxPacket.command == CMD_SET_LED_YELLOW ) {
			// overwrite current led value from the far end
			if( rxPacket.pl.u8[0] == 1 ) { setLedYellow( true ); } else { setLedYellow( false ); }
			if( rxPacket.payloadSize != 1 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_ANGLE_SETPOINT ) {
			// return angle setPoint value (this value is set from the far end)
			txPacket.responseType = RESP_ANGLE_SETPOINT;
			txPacket.payloadSize = PACKETSIZE_RESP_ANGLE_SETPOINT;
			txPacket.pl.s16[0] = getAngleSetPoint();
		} else if ( rxPacket.command == CMD_GET_PRIMARY_SETPOINT ) {
			// return primary setPoint (wheel motor or tacho offset) value (this value is set from the far end multiple time per second)
			txPacket.responseType = RESP_PRIMARY_SETPOINT;
			txPacket.payloadSize = PACKETSIZE_RESP_PRIMARY_SETPOINT;
			txPacket.pl.s16[0] = getPrimarySetPoint();
		} else if ( rxPacket.command == CMD_GET_PID_ANGLE_PROPERTIES ) {
			// get angle pid "constants", these are original set from far end
			txPacket.responseType = RESP_PID_ANGLE_PROPERTIES;
			txPacket.payloadSize = 12;
			txPacket.pl.u16[0] = getPidAngleProperties().p;
			txPacket.pl.u16[1] = getPidAngleProperties().i;
			txPacket.pl.u16[2] = getPidAngleProperties().d;
			txPacket.pl.u16[3] = (uint16_t) getPidAngleProperties().iTh;
			txPacket.pl.u32[2] = (uint32_t) getPidAngleProperties().iMax;
		} else if ( rxPacket.command == CMD_SET_PID_ANGLE_PROPERTIES ) {
			// set angle pid "constants" (adjust for maximal response / minimal overshoot)
			pidPropT tmp;
			tmp.p = rxPacket.pl.u16[0];
			tmp.i = rxPacket.pl.u16[1];
			tmp.d = rxPacket.pl.u16[2];
			tmp.iTh = rxPacket.pl.u16[3];
			tmp.iMax = rxPacket.pl.u32[2];
			setPidAngleProperties( tmp );
			if( rxPacket.payloadSize != 12 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_PID_PRIMARY_PROPERTIES ) {
			// get primary pid "constants", these are original set from far end
			txPacket.responseType = RESP_PID_PRIMARY_PROPERTIES;
			txPacket.payloadSize = 12;
			txPacket.pl.u16[0] = getPidPrimaryProperties().p;
			txPacket.pl.u16[1] = getPidPrimaryProperties().i;
			txPacket.pl.u16[2] = getPidPrimaryProperties().d;
			txPacket.pl.u16[3] = (uint16_t)getPidPrimaryProperties().iTh;
			txPacket.pl.u32[2] = (uint32_t)getPidPrimaryProperties().iMax;
		} else if ( rxPacket.command == CMD_SET_PID_PRIMARY_PROPERTIES ) {
			// set primary pid motor "constants" (adjust for maximal response / minimal overshoot)
			pidPropT tmp;
			tmp.p = rxPacket.pl.u16[0];
			tmp.i = rxPacket.pl.u16[1];
			tmp.d = rxPacket.pl.u16[2];
			tmp.iTh = rxPacket.pl.u16[3];
			tmp.iMax = rxPacket.pl.u32[2];
			setPidPrimaryProperties( tmp );
			if( rxPacket.payloadSize != 12 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_PWM_LIMIT ) {
			// get maximal motor power, this is original set from far end
			txPacket.responseType = RESP_PWM_LIMIT;
			txPacket.payloadSize = PACKETSIZE_RESP_PWM_LIMIT;
			txPacket.pl.u16[0] = getPwmLimit();
		} else if ( rxPacket.command == CMD_GET_PWM_MANUAL ) {
			// TODO: remove, not used anymore
			txPacket.responseType = RESP_PWM_MANUAL;
			txPacket.payloadSize = PACKETSIZE_RESP_PWM_MANUAL;
			txPacket.pl.s16[0] = 0;
		} else if ( rxPacket.command == CMD_SET_PWM_MANUAL ) {
			// TODO: remove, not used anymore
		} else if ( rxPacket.command == CMD_SET_PWM_LIMIT ) {
			// set maximal motor power
			setPwmLimit(rxPacket.pl.u16[0]);
			if( rxPacket.payloadSize != 2 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_SET_PWM_DELTA ) {
			// set maximal pwm increase per 2.5ms
			setPwmDelta(rxPacket.pl.u32[0]);
			if( rxPacket.payloadSize != 4 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_PWM_DELTA ) {
			// get maximal pwm increase per 2.5ms
			txPacket.responseType = RESP_PWM_DELTA;
			txPacket.payloadSize = 4;
			txPacket.pl.u32[0] = getPwmDelta();
		} else if ( rxPacket.command == CMD_GET_GAIN ) {
			// TODO: remove, not used anymore
			txPacket.responseType = RESP_GAIN;
			txPacket.payloadSize = PACKETSIZE_RESP_GAIN;
			txPacket.pl.u8[0] = 0;
		} else if ( rxPacket.command == CMD_SET_GAIN ) {
			// TODO: remove, not used anymore
			if( rxPacket.payloadSize != PACKETSIZE_CMD_SET_GAIN ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_MOTOR_TIMEOUT ) {
			// get the amount of periods after the motor will stop when no valid packed received, this value is original set from far end
			txPacket.responseType = RESP_MOTOR_TIMEOUT;
			txPacket.payloadSize = PACKETSIZE_RESP_MOTOR_TIMEOUT;
			txPacket.pl.u16[0] = getMotorTimeout();
		} else if ( rxPacket.command == CMD_SET_MOTOR_TIMEOUT ) {
			// set the amount of periods after the motor will stop when no valid packed received (safety function)
			setMotorTimeout( rxPacket.pl.u16[0] );
			if( rxPacket.payloadSize != 2 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_ANGLE_DIRECTION ) {
			// get the angle direction, this value is original set from far end
			txPacket.responseType = RESP_ANGLE_DIRECTION;
			txPacket.payloadSize = PACKETSIZE_RESP_ANGLE_DIRECTION;
			txPacket.pl.u8[0] = getAngleDirection();
		} else if ( rxPacket.command == CMD_SET_ANGLE_DIRECTION ) {
			// set the angle direction, or left or right ball handler shall have inverted angle behavior
			setAngleDirection( (bool)rxPacket.pl.u8[0] );
			if( rxPacket.payloadSize != 1 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_SET_DRV8301 ) {
			// configure the drv8301 motor driver
			setDrv8301( rxPacket.pl.u16[0], rxPacket.pl.u16[1] );
			if( rxPacket.payloadSize != 4 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_DRV8301 ) {
			// read back both status registers and both control registers of the drv8301 motor controller
			txPacket.responseType = RESP_DRV8301;
			txPacket.payloadSize = PACKETSIZE_RESP_DRV8301;
			drv8301Type drv8301 = getDrv8301();
			txPacket.pl.u16[0] = drv8301.status1;
			txPacket.pl.u16[1] = drv8301.status2;
			txPacket.pl.u16[2] = drv8301.getControl1;
			txPacket.pl.u16[3] = drv8301.getControl2;
			txPacket.pl.u16[4] = drv8301.setControl1;
			txPacket.pl.u16[5] = drv8301.setControl2;
		} else if ( rxPacket.command == CMD_SET_MODE ) {
			// set the mode of the board e.g. pid on encoder, pid on tacho or directly setpoint to pwm
			setMode( rxPacket.pl.u8[0] );
			if( rxPacket.payloadSize != 1 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else if ( rxPacket.command == CMD_GET_MODE ) {
			// read back the current selected mode
			txPacket.responseType = RESP_MODE;
			txPacket.payloadSize = PACKETSIZE_RESP_MODE;
			txPacket.pl.u8[0] = getMode( );
		} else if ( rxPacket.command == CMD_GET_ANGLE_TACHO_ZERO ) {
			txPacket.responseType = RESP_ANGLE_TACHO_ZERO;
			txPacket.payloadSize = PACKETSIZE_RESP_ANGLE_TACHO_ZERO;
			angleTachoT angleTachoZero = getAngleTachoZero();
			txPacket.pl.u16[0] = angleTachoZero.angle;
			txPacket.pl.u16[1] = angleTachoZero.tacho;
		} else if ( rxPacket.command == CMD_CLEAR_ERRORS ) {
			// clear the (reported) errors from far end
			clearCommunicationError(rxPacket.pl.u16[0]);
			clearEncoderError(rxPacket.pl.u16[1]);
			clearPidError(rxPacket.pl.u16[2]);
			clearPwmError(rxPacket.pl.u16[3]);
			clearSafetyError(rxPacket.pl.u16[4]);
			clearSchedulerError(rxPacket.pl.u16[5]);
			clearSpiMasterError(rxPacket.pl.u16[6]);
			if( rxPacket.payloadSize != 14 ) { error |= COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE; }
		} else {
			// invalid command received, will be reported in next cycle to far end
			error |= COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND;
		}
	}

	if( sofAlignmentCounter > 10 ) {
		error |= COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT;
		sofAlignmentCounter = 0;
	}

	if( txPacket.responseType == RESP_DEFAULT ) {
		bool globalError = false;
		uint16_t communicationError = getCommunicationError();
		uint16_t encoderError = getEncoderError();
		uint16_t pidError = getPidError();
		uint16_t pwmError = getPwmError();
		uint16_t safetyError = getSafetyError();
		uint16_t schedulerError = getSchedulerError();
		uint16_t spiMasterError = getSpiMasterError();

		if( communicationError != 0 || encoderError != 0 || pidError != 0 || pwmError != 0 ||
				safetyError != 0 || schedulerError != 0 || spiMasterError != 0 ) {
			globalError = true;
			ledYellowClearCounter = 255;
			if( communicationError != communicationErrorPrevious || encoderError != encoderErrorPrevious ||
					pidError != pidErrorPrevious || pwmError != pwmErrorPrevious || safetyError != safetyErrorPrevious ||
					schedulerError != schedulerErrorPrevious || spiMasterError != spiMasterErrorPrevious ) {
				// one of the error has changed, directly send the error
				lastErrorSend = 255; // maximal counter forces the error to be send immediately
			}
			// store the error values for the next loop
			communicationErrorPrevious = communicationError;
			encoderErrorPrevious = encoderError;
			pidErrorPrevious = pidError;
			pwmErrorPrevious = pwmError;
			safetyErrorPrevious = safetyError;
			schedulerErrorPrevious = schedulerError;
			spiMasterErrorPrevious = spiMasterError;
		}

		// so no data request performed by the far end
		// send the default package to the far end
		// in case an error occurred on the board send the error code, and then the default packet for a number cycles
		if( globalError && ( lastErrorSend > 120 ) ) { // roughly the error is re-send about every second
			// send the error code to the far end
			txPacket.responseType = RESP_ERROR;
			txPacket.payloadSize = PACKETSIZE_RESP_ERROR;
			txPacket.pl.u16[0] = getCommunicationError();
			txPacket.pl.u16[1] = getEncoderError();
			txPacket.pl.u16[2] = getPidError();
			txPacket.pl.u16[3] = getPwmError();
			txPacket.pl.u16[4] = getSafetyError();
			txPacket.pl.u16[5] = getSchedulerError();
			txPacket.pl.u16[6] = getSpiMasterError();
			lastErrorSend = 0; // next number of cycles we want to sent the default response packet instead of the error
		} else {
			// no specific response required to send back to PC, instead send the default response
			pidExportT pidPrimaryExport = getPidPrimaryExport(); // get atomic set of values from primary pid
			txPacket.payloadSize = PACKETSIZE_CMD_DEFAULT_RESPONSE;
			txPacket.pl.s32[0] = getEncoderData().displacement;
			txPacket.pl.s16[2] = pidPrimaryExport.velocity; // getEncoderData().velocity can be out of sync with the related pidError value; // caution: payload index
			txPacket.pl.s16[3] = pidPrimaryExport.pidError;
			txPacket.pl.s32[2] = pidPrimaryExport.integral; // caution: payload index
			txPacket.pl.s32[3] = pidPrimaryExport.result32;
			txPacket.pl.s16[8] = getPwmValue(); // caution: payload index
			txPacket.pl.u32[5] = getSchedulerTime().measure;
			txPacket.pl.u32[6] = getSchedulerTime().calculation;
			txPacket.pl.u16[14] = getSafety().currentA;
			txPacket.pl.u16[15] = getSafety().currentB;
			txPacket.pl.u16[16] = getSafety().boardVoltage;
			txPacket.pl.u16[17] = getSafety().boardTemp;
			txPacket.pl.u16[18] = getSafety().tacho;
			txPacket.pl.u16[19] = getSafety().angle; // or motor temperature
			if( lastErrorSend < 255 ) { lastErrorSend++; } // if the error is not cleared, the error shall be re-send to the pc after a while
		}
	}

	sendPacket();
	if( ledGreenBlinkCounter < 200 ){
		ledGreenBlinkCounter++;
	} else {
		ledGreenBlinkCounter = 0;
		setLedGreen( ! getLedGreen() );
	}

	if( ledYellowClearCounter == 0 ) {
		setLedYellow(false); // clear yellow led error status
	} else {
		ledYellowClearCounter--;
		setLedYellow(true); // show the error on the yellow led
	}
}


// Every cycle of the communication task a packet will be send to the far end
void sendPacket() {
	static uint8_t txPacketAckId = 0;
	txPacket.ackId = txPacketAckId++; // wrap around
	txPacket.feedbackId = lastrxPackedAckId; // return last valid received ackId from sender, so with the bufferSpace the sender can accurate calculate the board worst case Rx Buffer space
	txPacket.bufferSpace = receiveBufferSpace();// inform the far end about the maximal package it can send

	uint16_t ii = 0; // use uint16_t because packet size = header + payload which is greater then 255
 	uint16_t txPacketSize = txPacket.payloadSize + FROM_BOARD_PACKET_HEADER_SIZE;

 	// Calculate the checksum
	uint8_t checksum = 0;
	txPacket.checksum = 0; // be sure the checksum is calculated without a checksum value
	char *txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
	for( ii = 0; ii < txPacketSize; ii++ ) { // iterate through all the bytes including the checksum itself (which was reset to 0)
		checksum += *txPacketPointer; // use simple add as checksum (instead of e.g. crc8)
		txPacketPointer++;
	}
	txPacket.checksum = checksum; // store the calculated checksum in the packet

	// Send the bytes through the serial port to the far end
	txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
	for( ii = 0; ii < txPacketSize; ii++ ) { // iterate through all the bytes
	 	if( ! sendByte( *txPacketPointer ) ) { error |= COMMUNICATION_ERROR_FROM_BOARD_WRITE;	}
		txPacketPointer++; // set the pointer to the next byte of the txPacket
	 	if( ii >= (txPacket.payloadSize + FROM_BOARD_PACKET_HEADER_SIZE) ) { error |= COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE; }
	}
 	if( txPacket.endCheck != 0x97 ) { error |= COMMUNICATION_ERROR_FROM_BOARD_END_CHECK; }
 	if( txPacket.sof != SOF ) { error |= COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN; }
}

// Whenever there is data available in the receive buffer the bytes will be used to assemble a complete packet.
// Collection of bytes to assemble the packet starts when the start of frame has been found.
// Then it will continue with adding bytes to the packet until the amount bytes is the same as the
// packet size (header + payload size) provided from the far end.
// When the packet is complete the processRxPacket will be activated, and the cycle starts over.
void receivePacket() {
	// TODO: make packet rxPacketTmp local static for this function instead of global, however then a initialization solution has to be found
	static uint8_t state = 0; // state 0 = search SOF, state 1 = complete the packet
	static uint16_t rxBytes = 0; // packet size = header + payload > 256 bytes
	static char *rxPacketTmpPointer = 0; // for efficient iterating through the rx packet

	uint8_t rxByte = receiveByte(); // TODO: investigate if we can do something with a read timeout e.g. when packet is not complete
	switch (state){
	case 0: // state: search SOF
		if( rxByte == SOF ){
			state = 1; // sof received, now start filling the buffer
			// Note: do not need to store sof in rxPacketTmp because it will always be the same
			rxPacketTmpPointer = (char *) &rxPacketTmp; // use char pointer to index the individual bytes of the rxPacketTmp
			rxPacketTmpPointer++; // move pointer to the next element in rx packet because first byte (sof) stored
			rxBytes = 1; // already 1 byte received
		} else {
			sofAlignmentCounter++; // monitor how many bytes have been processed before the start of frame was found
		}
		break;

	case 1:	// state: complete the packet
		rxBytes++; // increment amount of received bytes
		if( rxBytes <= TO_BOARD_PACKET_SIZE ) {
			*rxPacketTmpPointer = rxByte; // store received byte in rx packet
			rxPacketTmpPointer++; // move pointer to the next location of the rx packet (to store the next byte)
			if( rxBytes >= 4 ) { // TODO: replace by e.g. (rxPacketTmp.payloadSize-rxPacketTmp.sof)
				// size of payload (provided by far end) is available
				if( rxBytes == ((uint16_t)rxPacketTmp.payloadSize) + TO_BOARD_PACKET_HEADER_SIZE ) {
					// received the last byte of current packet, packet complete, all done
					memcpy((void *) &rxPacket, (void *) &rxPacketTmp, rxPacketTmp.payloadSize + TO_BOARD_PACKET_HEADER_SIZE );
					if( processRxPacket ) { error |= COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN; }
					processRxPacket = true; // set flag to inform the packet is complete
					state = 0; // start searching for the next packet
					if( rxPacketTmp.endCheck != 0x98 ) { // check if rx buffer was written out of range
						error |= COMMUNICATION_ERROR_TO_BOARD_END_CHECK;
						rxPacketTmp.endCheck = 0x98; // reset to default
					}
				} else if( rxBytes > ((uint16_t)rxPacketTmp.payloadSize) + TO_BOARD_PACKET_HEADER_SIZE ) {
					// more bytes received then set by the far end through the payload size
					error |= COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW;
					state = 0; // reject current packet and start searching for new packet
				}
			}
		} else {
			// rx packet (buffer) full, should never happen
			error |= COMMUNICATION_ERROR_TO_BOARD_OVERFLOW;
			state = 0; // reject current packet and start searching for new packet
		}
		break;
	default:
		break;
	}
}

/*! \brief Check if the ack id of every received packet increases by one
 */
void rxPacketAckIdCheck() {
	static uint8_t ackId = 0;
	if( ackId != rxPacket.ackId ) {
		// the ack id out of sync error will appear when the far end sender is restarted
		error |= COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC;
	}
	ackId = rxPacket.ackId + 1;
}

/*! \brief Check if calculated packet checksum is the same as the checksum provided by the far end
 */
bool rxPacketChecksumFail() {
	uint8_t checksum = 0;
	uint16_t ii = 0;
	static uint8_t checksumValidCnt = 0;
	bool returnValue = false;
	char *rxPacketPointer = (char *) &rxPacket; // use char pointer to index the individual bytes of the rxPacket
	for( ii = 0; ii < ( TO_BOARD_PACKET_HEADER_SIZE + rxPacket.payloadSize ); ii++ ) {
		checksum += *rxPacketPointer;
		rxPacketPointer++;
	}
	checksum -= rxPacket.checksum; // checksum added in above loop, but checksum itself should be excluded from check calculation

	if( checksum == rxPacket.checksum ) {
		lastrxPackedAckId = rxPacket.ackId; // use ackId to send back as feedback to PC
		if( checksumValidCnt < 5 ) {
			checksumValidCnt++;
		} else {
			validPacketRecieved(); // only enable motor if 5 valid packets have been received
		}
	} else {
		checksumValidCnt = 0;
		error |= COMMUNICATION_ERROR_TO_BOARD_CHECKSUM;
		returnValue = true;
	}
	return returnValue;
}


uint16_t getCommunicationError() {
	return error;
}

void clearCommunicationError( uint16_t value ){
	error &= ~value; // clear only the bits set in value
}

void setRxBufferOverflowError(){
	error |= COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW;
}
