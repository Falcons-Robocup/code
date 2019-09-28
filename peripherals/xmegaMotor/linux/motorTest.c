// Copyright 2015, 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#include <fcntl.h>
#include <getopt.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h> // for memset
#include <termios.h>
#include <unistd.h>
#include <inttypes.h>
#include "communication.h"

static toBoardPacketT txPacket;
static fromBoardPacketT rxPacket, rxPacketTmp; // use send packet definition from board as receive packet for pc

static int port = 0; // handle to serial interface
static int farEndBufferSize = 0;
static int bytesSendDuringCurrentfarEndBufferSizeRefresh = 0; // use to calculate the worst case far end receive buffer size
static int bytesSendDuringMin1farEndBufferSizeRefresh = 0; // use to calculate the worst case far end receive buffer size
static int bytesSendDuringMin2farEndBufferSizeRefresh = 0; // use to calculate the worst case far end receive buffer size
static uint8_t sofAlignmentCounter = 0;
static int farEndAckIdMis = 0;
static int farEndChecksumFail = 0;
static int receiveAckIdMis = 0;
static int receiveChecksumFail = 0;
static _Bool ledYellow = 0;
static long unsigned int sendPacketsCnt = 0;
static long unsigned int receivedPacketsCnt = 0;
static _Bool currentCalibrate = false;
static uint8_t mode = MODE_AA_DO_NOT_USE_ZERO;
static uint16_t angleZero = 0;
static uint16_t tachoZero = 0;
static uint16_t applicationId = 255;
static int16_t primarySetPoint = 0; // keep track of the setPoint that was send to the board for printing

// enable next two when displacement / velocity / pidError are printed in m/s instead of encoder ticks
// static double oneWheelTick = 0.317 / ( 1.0 * 12 * 512 * 4 ); // 0.317 meter = wheel circumference robot 6, 12 = gearbox, 512 = encoder, 4 = quadrature
// static double oneTimeTick = 0.002; // velocity is measured very 2ms

// print one send packet on one line
void printSendPacket( ){
	if( txPacket.command == CMD_LOOPBACK ||
		txPacket.command == CMD_AA_GET_BOARD ||
		txPacket.command == CMD_CLEAR_ERRORS ||
		txPacket.command == CMD_GET_LED_YELLOW ||
	 	txPacket.command == CMD_GET_MOTOR_TIMEOUT ||
	 	txPacket.command == CMD_GET_DRV8301 ||
	 	txPacket.command == CMD_GET_MODE ||
	 	txPacket.command == CMD_SET_MODE ||
	 	txPacket.command == CMD_GET_ANGLE_TACHO_ZERO ||
	 	txPacket.command == CMD_GET_PID_ANGLE_PROPERTIES ||
		txPacket.command == CMD_SET_PID_ANGLE_PROPERTIES ||
	 	txPacket.command == CMD_GET_PID_PRIMARY_PROPERTIES ||
		txPacket.command == CMD_SET_PID_PRIMARY_PROPERTIES ||
		txPacket.command == CMD_GET_PWM_LIMIT ||
		txPacket.command == CMD_SET_PWM_LIMIT ||
		txPacket.command == CMD_GET_PWM_DELTA ||
		txPacket.command == CMD_SET_PWM_DELTA ||
		txPacket.command == CMD_GET_ANGLE_DIRECTION ||
		txPacket.command == CMD_SET_ANGLE_DIRECTION ||
		txPacket.command == CMD_SET_LED_YELLOW ) {
		// for these packets we do not need to inspect the transmitted packet
	} else if( txPacket.command == CMD_SET_ANGLE_SETPOINT ) {
 		// printf("send    : set angle setPoint to %5d\n", txPacket.pl.s16[0] );
	} else if( txPacket.command == CMD_SET_PRIMARY_SETPOINT ) {
 		// printf("send    : set wheel speed (or tacho offset) to %5d = %4.3f m/s\n", txPacket.pl.s16[0], txPacket.pl.s16[0]*oneWheelTick/oneTimeTick);
		primarySetPoint = txPacket.pl.s16[0];
	} else if( txPacket.command == CMD_SET_MOTOR_TIMEOUT ) {
 		// printf("send    : set motor timeout %4.0f ms\n", txPacket.pl.u16[0] * 2.5);
	} else if( txPacket.command == CMD_SET_MODE ) {
		if( txPacket.pl.u8[0] == MODE_PID_ANGLE ) {	printf( "send    : mode angle = first pid input is angle + setPoint, second pid input is first pid + tacho\n" ); }
		else if( txPacket.pl.u8[0] == MODE_PID_ENCODER ) { printf( "send    : mode encoder = pid input is encoder + setPoint\n" ); }
		else if( txPacket.pl.u8[0] == MODE_PID_TACHO ) { printf( "send    : mode tacho = pid input is tacho + setPoint\n" ); }
		else if( txPacket.pl.u8[0] == MODE_PWM_ONLY ) { printf( "send    : mode pwm only = no pid, pwm is directly controlled from setPoint\n" ); }
		else if( txPacket.pl.u8[0] == MODE_COMMUNICATION_TIMEOUT ) { printf( "send    : mode timeout = motor disabled because communication timeout\n" ); }
		else if( txPacket.pl.u8[0] == MODE_DISABLE_MOTOR_POWER ) { printf( "send    : mode disable = motor disabled (after reset / or Linux software initialization)\n" ); }
		else if( txPacket.pl.u8[0] == MODE_ZERO_CURRENT_CALIBRATION ) { printf( "send    : mode calibration = motor disabled for zero current calibration of drv8301\n" ); }
		else { printf( "ERROR   : no mode set %d, abort\n", rxPacket.pl.u8[0] ); exit(1); }
	} else if( txPacket.command == CMD_SET_DRV8301 ) {
		uint16_t control1 = txPacket.pl.u16[0];
		printf( "send    : drv8301 control 1 (0x%04x):", control1 );
		if( ((control1>>0) & 3) == 0 ) { printf( " gate 1.7A" ); }
		else if( ((control1>>0) & 3) == 1 ) { printf( " gate 0.7A" ); }
		else if( ((control1>>0) & 3) == 2 ) { printf( " gate 0.25A" ); }
		else { printf( " WRONG gate drive current SHALL NEVER HAPPEN" );}
		if( ((control1>>2) & 1) == 1 ) { printf( ", reset gate driver latched faults (reverts to 0)" ); }
		if( ((control1>>3) & 1) == 1 ) { printf( ", 3 PWM mode" ); } else { printf (", 6 PWM mode" ); }
		if( ((control1>>4) & 3) == 0 ) { printf( ", overcurrent protection mode current limit" ); }
		else if( ((control1>>4) & 3) == 1 ) { printf( ", OCP mode OC last shut down" ); }
		else if( ((control1>>4) & 3) == 2 ) { printf( ", OCP mode report only" ); }
		else { printf( ", OCP mode OC disable" ); }
		if( ((control1>>6) & 0x1f) == 0 ) { printf( ", OC Vds 0.060V" ); }
		else if( ((control1>>6) & 0x1f) == 1 ) { printf( ", OC Vds 0.068V" );	}
		else if( ((control1>>6) & 0x1f) == 2 ) { printf( ", OC Vds 0.076V" ); }
		else if( ((control1>>6) & 0x1f) == 3 ) { printf( ", OC Vds 0.086V" );	}
		else if( ((control1>>6) & 0x1f) == 4 ) { printf( ", OC Vds 0.097V" ); }
		else if( ((control1>>6) & 0x1f) == 5 ) { printf( ", OC Vds 0.109V" ); }
		else if( ((control1>>6) & 0x1f) == 6 ) { printf( ", OC Vds 0.123V" ); }
		else if( ((control1>>6) & 0x1f) == 7 ) { printf( ", OC Vds 0.138V" ); }
		else if( ((control1>>6) & 0x1f) == 8 ) { printf( ", OC Vds 0.155V" ); }
		else if( ((control1>>6) & 0x1f) == 9 ) { printf( ", OC Vds 0.175V" ); }
		else if( ((control1>>6) & 0x1f) == 10 ) { printf( ", OC Vds 0.197V" ); }
		else if( ((control1>>6) & 0x1f) == 11 ) { printf( ", OC Vds 0.222V" ); }
		else if( ((control1>>6) & 0x1f) == 12 ) { printf( ", OC Vds 0.250V" ); }
		else if( ((control1>>6) & 0x1f) == 13 ) { printf( ", OC Vds 0.282V" ); }
		else if( ((control1>>6) & 0x1f) == 14 ) { printf( ", OC Vds 0.317V" ); }
		else if( ((control1>>6) & 0x1f) == 15 ) { printf( ", OC Vds 0.358V" ); }
		else if( ((control1>>6) & 0x1f) == 16 ) { printf( ", OC Vds 0.403V" ); } // this is the drv8301 power-up default
		else if( ((control1>>6) & 0x1f) == 17 ) { printf( ", OC Vds 0.454V" ); }
		else if( ((control1>>6) & 0x1f) == 18 ) { printf( ", OC Vds 0.511V" ); }
		else if( ((control1>>6) & 0x1f) == 19 ) { printf( ", OC Vds 0.576V" ); }
		else if( ((control1>>6) & 0x1f) == 20 ) { printf( ", OC Vds 0.648V" ); }
		else if( ((control1>>6) & 0x1f) == 21 ) { printf( ", OC Vds 0.730V" ); }
		else if( ((control1>>6) & 0x1f) == 22 ) { printf( ", OC Vds 0.822V" ); }
		else if( ((control1>>6) & 0x1f) == 23 ) { printf( ", OC Vds 0.926V" ); }
		else if( ((control1>>6) & 0x1f) == 24 ) { printf( ", OC Vds 1.043V" ); }
		else if( ((control1>>6) & 0x1f) == 25 ) { printf( ", OC Vds 1.175V" ); }
		else if( ((control1>>6) & 0x1f) == 26 ) { printf( ", OC Vds 1.324V" ); }
		else if( ((control1>>6) & 0x1f) == 27 ) { printf( ", OC Vds 1.491V" ); }
		else if( ((control1>>6) & 0x1f) == 28 ) { printf( ", OC Vds 1.679V" ); }
		else if( ((control1>>6) & 0x1f) == 29 ) { printf( ", OC Vds 1.892V" ); }
		else if( ((control1>>6) & 0x1f) == 30 ) { printf( ", OC Vds 2.131V" ); }
		else { printf( ", OC Vds 2.400V" ); }
		printf( "\n" );

		uint16_t control2 = txPacket.pl.u16[1];
		printf( "send    : drv8301 control 2 (0x%04x):", control2 );
		if( ((control2>>0) & 3) == 0 ) { printf( " report OT and OC" ); }
		else if( ((control2>>0) & 3) == 1 ) { printf( " report OT only" ); }
		else if( ((control2>>0) & 3) == 2 ) { printf( " report OC only" ); }
		else { printf( " report reserved SHALL NEVER HAPPEN" ); }
		if( ((control2>>2) & 3) == 0 ) { printf( ", shunt 10V/V" ); }
		else if( ((control2>>2) & 3) == 1 ) { printf( ", shunt 20V/V" ); }
		else if( ((control2>>2) & 3) == 2 ) { printf( ", shunt 40V/V" ); }
		else { printf( ", shunt 80V/V" ); }
		if( ((control2>>4) & 1) == 1 ) { printf( ", external calibration channel 1" ); }
		if( ((control2>>5) & 1) == 1 ) { printf( ", external calibration channel 2" ); }
		if( ((control2>>6) & 1) == 1 ) { printf( ", OC_TOFF off-time control" ); } else { printf( ", OC_TOFF cycle by cycle" ); }
		printf( "\n" );
 	} else {
		int packetSize = txPacket.payloadSize + TO_BOARD_PACKET_HEADER_SIZE;
 		printf( "send    : %3d %3d %3d %3d | ", farEndAckIdMis, farEndChecksumFail, packetSize, farEndBufferSize );

 		char *txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
 		for( int ii = 0; ii < packetSize; ii++) {
 			printf( "%02x ", (uint8_t) *txPacketPointer );
 			if( ii == (TO_BOARD_PACKET_HEADER_SIZE-1) ) { printf( "   | " ); }
 			txPacketPointer++;
 		}
 		printf("\n");
 	}
}

// The send packet to board
// Each packet send to the board should result in a packet back
// The payload data and size is depended on the received command.
// The send packet function is called as a response on the received package.
// Each valid received package should generate one send package.
// The payload data and size is depended on the received command.
void sendPacket( ) {
 	int packetSize = txPacket.payloadSize + TO_BOARD_PACKET_HEADER_SIZE;
 	// because of the asynchronous communication it can not be guaranteed that the previous send packet is
 	// already part of the last received farEndBufferSize, to be sure we do not overwrite the rx buffer on
 	// the far end we add up all bytes send between the last 3 received packets.
 	int worstCaseFarEndAvailableSize = farEndBufferSize - bytesSendDuringMin1farEndBufferSizeRefresh - bytesSendDuringMin2farEndBufferSizeRefresh - bytesSendDuringCurrentfarEndBufferSizeRefresh;
 	if( worstCaseFarEndAvailableSize < packetSize ) {
 		printf("do not send packet: packet size %3d, worst case %3d, far end buffer size %3d, min1 %3d, min2 %3d, current send %3d\n", packetSize, worstCaseFarEndAvailableSize, farEndBufferSize, bytesSendDuringMin1farEndBufferSizeRefresh, bytesSendDuringMin2farEndBufferSizeRefresh, bytesSendDuringCurrentfarEndBufferSizeRefresh );
 	} else {
 	 	txPacket.sof = SOF; // always the same

 	 	static uint8_t ackId = 0; // first packet start with ackId 0, then increment (wrap around) for each following packet
 	 	txPacket.ackId = ackId++; // increment for each packet (wrap around at 0xff)

 	 	// Calculate the checksum
 	 	txPacket.checksum = 0; // for new packet, checksum starts at 0
 		char checksum = 0;
 		// TODO: check if pointer assignment below is correct
 		char *txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
 		for( int ii = 0; ii < packetSize; ii++ ) { // iterate through all the bytes including the checksum itself (which was reset to 0)
 			checksum += *txPacketPointer; // use simple add as checksum (instead of e.g. crc8)
 			txPacketPointer++; // set the pointer to the next byte of the txPacket
 	 	}
 		txPacket.checksum = checksum; // store the calculated checksum in the packet
 		// send packet to the far end
 		sendPacketsCnt++;
 	 	int numBytes = write( port, &txPacket, packetSize);
 		if( numBytes != packetSize ) { printf("ERROR   : incorrect number of bytes written %d %d\n", numBytes, packetSize); exit(1); }

 		printSendPacket();
 		bytesSendDuringCurrentfarEndBufferSizeRefresh += packetSize;
 		// printf("do     send packet: packet size %3d, worst case %3d, far end buffer size %3d, min1 %3d, min2 %3d, current send %3d\n", packetSize, worstCaseFarEndAvailableSize, farEndBufferSize, bytesSendDuringMin1farEndBufferSizeRefresh, bytesSendDuringMin2farEndBufferSizeRefresh, bytesSendDuringCurrentfarEndBufferSizeRefresh );
 	}
}

void sendLoopBackPacket( ) {
	txPacket.command = CMD_LOOPBACK;
	txPacket.payloadSize = 63 - TO_BOARD_PACKET_HEADER_SIZE; // the buffer on the far end can only hold 255 bytes
	// txPacket.payloadSize = 2; // setPoint is only 2 bytes
	for( int ii = 0; ii < txPacket.payloadSize; ii++ ){
		txPacket.pl.u8[ii] = ii;
	}
	sendPacket( );
}

// function to convert the raw adc value from the xmega to ADC voltage
float adcToVoltage( float adc ) {
	// the adc is 12 bits, range 0 to 4095
	// the adc works with an offset (to be able to detect zero crossing)
	// the vRef is 2.5v
	// each bit is 2.5v/4095 = 0.61mV
	// 4095 = vRef - deltaV = vRef - (0.05 * vRef) = 0.95 * vRef = 2.375V
	// 205 = 0.05 * 4095 = 0V
	// 0 = - deltaV = - deltaV = 0.05 * vRef = -0.125V (not sure the ADC will return 0)
	return ( 2.5 * adc / 4095.0 ) - 0.125;
}

// function to convert the adc voltage to the board voltage
float voltageToPowerSuppy( float adcVoltage ) {
	return adcVoltage * ( 3.0 + 30.0 ) / 3.0; // values from schematic
}

// function to convert the adc voltage to ntc resistance
float voltageToNtcResistance( float adcVoltage ) {
	// for as well board temperature NTC as motor NTC temperature
	// the pull up resistor is 4k7 and connected to 3v3
	// the adcVoltage is measured over the ntc resistor
	float vcc3v3 = 3.27;
	float pullUp = 4700;
	// return current resistance value of ntc resistor
	return adcVoltage * pullUp / ( vcc3v3 - adcVoltage );
}

// function to convert the board ntc resistance to board temperature
float resistanceToBoardTemperature( float ntcResistance ) {
	// regarding Veds the board contains a Panasonic ERT-J1VT472J
	// nominal resistance at 25C is 4k7 +/- 5%, B value at 25/50K 4500K +/- 2%
	const float ntcResistanceAt25Degrees = 4700;
	const float ntcNominalTemperature = 25;
	const float ntcBCoefficient = 4500;
	// Using Steinhart-Hart formula
	// ambientTemp = 1 / ( ln( ntcR/ntcNomR)/ntcBcoef + 1/nominalTemp )
	float kelvin = 1.0 / ( log( ntcResistance / ntcResistanceAt25Degrees ) / ntcBCoefficient + 1.0 / ( ntcNominalTemperature + 273.15 ) );
	float celcius = kelvin - 273.15;
	return celcius;
}

// function to convert the adc voltage to motor temperature temperature using 3950 10k 1% ntc
float resistanceToMotorTemparature( float ntcResistance ) {
	// pull up to 3v3 is 4k7
	const float ntcResistanceAt25Degrees = 5000;
	const float ntcNominalTemperature = 25;
	const float ntcBCoefficient = 3500; // 3950 gives a to low value;
	// Using Steinhart-Hart formula
	// ambientTemp = 1 / ( ln( ntcR/ntcNomR)/ntcBcoef + 1/nominalTemp )
	float kelvin = 1.0 / ( log( ntcResistance / ntcResistanceAt25Degrees ) / ntcBCoefficient + 1.0 / ( ntcNominalTemperature + 273.15 ) );
	float celcius = kelvin - 273.15;
	return celcius;
}


// function to convert the adc voltage to motor current
float voltageToCurrent( float offset, float voltage, float correction ) {
	// shunt resistor is 1mOhm
	// TODO: add way to select between wheel motor board (shunt 1mOhm) and ball handler board (shunt 10mOhm)
	// e.g. 10A
	// 10 * 0.001 = 10mV
	// gain of 40 => 40 * 10mV = 400mV
	// current = voltage / ( gain * shunt)
	// there is still a big error in the current, it is unclear where this comes from
	// for the time being calibrate between actual current and board current measurement
	float current = correction * ( offset - voltage ) / ( 40.0 * 0.001 );
	return current;
}

// print one received packet on one line
void printReceivedPacket(){
	int packetSize = FROM_BOARD_PACKET_HEADER_SIZE + rxPacket.payloadSize;
	static int defaultRespCounter = 0; // we do not want to print all default responses
	static float currentAOffset = 0; // value of channel A when no current running (motor not active)
	static float currentBOffset = 0; // value of channel B when no current running (motor not active)
	static float currentATotal = 0; // value of channel A when no current running (motor not active)
	static float currentBTotal = 0; // value of channel B when no current running (motor not active)
	static size_t calibrateLoops = 0;
	static float currentAMeasure = 0; // average out current A to get rid of the noise
	static float currentBMeasure = 0; // average out current B to get rid of the noise
	static size_t currentMeasureCount = 0;

	if( rxPacket.responseType == RESP_LED_YELLOW ) {
		if( rxPacket.pl.u8[0] == 1 ) {
			printf( "INFO    : yellow led is on\n");
			ledYellow = true;
		} else {
			printf( "INFO    : yellow led is off\n");
			ledYellow = false;
		}
	} else if ( rxPacket.responseType == RESP_DEFAULT ) {
		if( rxPacket.payloadSize != 40 ) {
			printf( "ERROR   : default response packet size is %d but should be %d\n", rxPacket.payloadSize, 38 ); exit(1);
		} else {
			if( currentCalibrate ) {
				if( calibrateLoops < 100 ) {
					// wait a while before everything is settled, the calibration value was not always stable
					calibrateLoops++; // keep track how many times the calibration has been performed
				} else if( calibrateLoops < 200 ) {
					currentATotal += rxPacket.pl.u16[14]; // sum of all current A;
					currentBTotal += rxPacket.pl.u16[15]; // sum of all current B;
					calibrateLoops++; // keep track how many times the calibration has been performed
				} else {
					currentCalibrate = false; // inform main loop we are done with calibration
					currentAOffset = currentATotal / (calibrateLoops - 100); // calculate the average of current A when motor not running
					currentBOffset = currentBTotal / (calibrateLoops - 100); // calculate the average of current B when motor not running
					printf("INFO    : calibration current input A %.3f V, current input B %.3f V\n", adcToVoltage( currentAOffset ), adcToVoltage( currentBOffset ) );
					calibrateLoops = 0; // for next calibration round
					currentATotal = 0;
					currentBTotal = 0;
				}
			}
			currentAMeasure += rxPacket.pl.u16[14]; // sum of all current A;
			currentBMeasure += rxPacket.pl.u16[15]; // sum of all current B;
			currentMeasureCount++;
			if( defaultRespCounter == 100 ) { // prevent cluttering up screen
				defaultRespCounter = 0;
				float currentA = voltageToCurrent( adcToVoltage( currentAOffset ), adcToVoltage( 1.0 * currentAMeasure/currentMeasureCount), 0.512 ); // current correct A measured
				if( currentA < 0 ) { currentA = 0; }
				float currentB = voltageToCurrent( adcToVoltage( currentBOffset ), adcToVoltage( 1.0 * currentBMeasure/currentMeasureCount), 0.570 ); // current correction B measured
				if( currentB < 0 ) { currentB = 0; }
				currentAMeasure = 0;
				currentBMeasure = 0;
				currentMeasureCount = 0;
				float powerSupplyVoltage = voltageToPowerSuppy( adcToVoltage( rxPacket.pl.u16[16] ) );
				float boardNtcResistance = voltageToNtcResistance( adcToVoltage( rxPacket.pl.u16[17] ));
				float boardTemperature = resistanceToBoardTemperature ( boardNtcResistance );
				float motorNtcResistance = voltageToNtcResistance( adcToVoltage( rxPacket.pl.u16[19] ));
				float motorTemperature = resistanceToMotorTemparature( motorNtcResistance );
				if( mode == MODE_PID_ENCODER ) {
					// printf( "INFO    : displacement %8.2f m, velocity %6.3f m/s pidError %6.3f m/s, integral %7d, pid %10d, pwm %6d, timeM %6d, timeC %6d, curA %4.2f, curB %4.2f, volt %3.2f, tempB %4.2f, tempM %4.2f\n",
					printf( "INFO    : disp %8d set %5d m, vel %5d, pidError %5d, intgrl %7d, pid %10d, pwm %6d, timeM %6d, timeC %6d, curA %4.2f, curB %4.2f, volt %3.2f, tempB %4.2f, tempM %4.2f\n",
						rxPacket.pl.s32[0], // *oneWheelTick, // wheel displacement
						primarySetPoint,
						rxPacket.pl.s16[2], // *oneWheelTick/oneTimeTick, // wheel velocity
						rxPacket.pl.s16[3], // *oneWheelTick/oneTimeTick, // wheel velocity error
						rxPacket.pl.s32[2], // pid integral value, caution: payload index
						rxPacket.pl.s32[3], // pid result
						rxPacket.pl.s16[8], // pwm value
						rxPacket.pl.u32[5], // measure time
						rxPacket.pl.u32[6], // calculation time
						currentA, // u16[14]
						currentB, // u16[15]
						powerSupplyVoltage, // u16[16]
						boardTemperature, // board temperature
						// adcToVoltage( rxPacket.pl.u16[18]), // tacho
						// rxPacket.pl.u16[18], // tacho
						motorTemperature ); // motor temperature
				} else {
					printf( "INFO    : pidError %4d, integral %7d, pid %10d, pwm %6d, timeM %6d, timeC %6d, curA %4.2f, curB %4.2f, volt %3.2f, tempB %4.2f, tacho %4.2f %5d, angle %4.2f %5d\n",
						rxPacket.pl.s16[3], // tacho velocity error
						rxPacket.pl.s32[2], // pid integral value, caution: payload index
						rxPacket.pl.s32[3], // pid result
						rxPacket.pl.s16[8], // pwm value
						rxPacket.pl.u32[5], // measure time
						rxPacket.pl.u32[6], // calculation time
						currentA, // u16[14]
						currentB, // u16[15]
						powerSupplyVoltage, // u16[16]
						boardTemperature, // board temperature
						adcToVoltage( rxPacket.pl.u16[18]), // tacho
						rxPacket.pl.u16[18], // tacho
						adcToVoltage( rxPacket.pl.u16[19]), // angle
						rxPacket.pl.u16[19] ); // angle
				}
			} else {
				defaultRespCounter++;
			}
		}
	} else if ( rxPacket.responseType == RESP_LOOPBACK ) {
		for( int ii = 0; ii < rxPacket.payloadSize; ii++ ) {
			if( ii != rxPacket.pl.u8[ii] ) {
				printf( "ERROR   : loopback value is 0x%02x but should be 0x%02x\n", rxPacket.pl.u8[ii], ii ); exit(1);
			}
		}
	} else if ( rxPacket.responseType == RESP_MOTOR_TIMEOUT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "ERROR   : invalid payload size for motor timeout response %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf( "INFO    : motor will be disabled if no valid packet has been received within %.1f ms\n", 2.5 * rxPacket.pl.u16[0] );
		}
	} else if ( rxPacket.responseType == RESP_PWM_LIMIT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "ERROR   : invalid payload size for pwm limit response %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf( "INFO    : pwm will be limited to %d\n", rxPacket.pl.u16[0] );
		}
	} else if ( rxPacket.responseType == RESP_PWM_DELTA ) {
		if( rxPacket.payloadSize != 4 ) {
			printf( "ERROR   : invalid payload size for pwm delta response %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf( "INFO    : pwm delta (maximal increase per 2.5ms) is limited to %d (%.2f%% per second)\n", rxPacket.pl.u32[0], 400.0*rxPacket.pl.u32[0]/(648*65536.0) );
		}
	} else if ( rxPacket.responseType == RESP_PID_ANGLE_PROPERTIES ) {
		if( rxPacket.payloadSize != 12 ) {
			printf( "ERROR   : invalid payload size for the pid angle properties %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf( "INFO    : pid angle value are set to: Kp %d, Ki %d, Kd %d iTh %d, iMax %d\n", rxPacket.pl.u16[0], rxPacket.pl.u16[1], rxPacket.pl.u16[2], rxPacket.pl.u16[3], rxPacket.pl.u32[2] );
		}
	} else if ( rxPacket.responseType == RESP_PID_PRIMARY_PROPERTIES ) {
		if( rxPacket.payloadSize != 12 ) {
			printf( "ERROR   : invalid payload size for the pid primary properties %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf( "INFO    : pid primary value are set to: Kp %d, Ki %d, Kd %d iTh %d, iMax %d\n", rxPacket.pl.u16[0], rxPacket.pl.u16[1], rxPacket.pl.u16[2], rxPacket.pl.u16[3], rxPacket.pl.u32[2] );
		}
	} else if ( rxPacket.responseType == RESP_ANGLE_DIRECTION ) {
		if( rxPacket.payloadSize != 1 ) {
			printf( "ERROR   : invalid payload size for the angle direction %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			if( rxPacket.pl.u8[0] == 0 ) {
				printf( "INFO    : angle direction behavior NOT inverted\n" );
			} else {
				printf( "INFO    : angle direction behavior inverted\n" );
			}
		}
	} else if ( rxPacket.responseType == RESP_MODE ) {
		if( rxPacket.payloadSize != 1 ) {
			printf( "ERROR   : invalid payload size for the mode %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			mode = rxPacket.pl.u8[0]; // use lateron in this application to enable or disable functions
			if( rxPacket.pl.u8[0] == MODE_PID_ANGLE ) { printf( "INFO    : mode angle = first pid input is angle + setpoint, second pid input is first pid + tacho\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_PID_ENCODER ) { printf( "INFO    : mode encoder = pid input is encoder + setpoint\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_PID_TACHO ) { printf( "INFO    : mode tacho = pid input is tacho + setpoint\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_PWM_ONLY ) { printf( "INFO    : mode pwm = no pid, pwm is directly controlled from setpoint\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_COMMUNICATION_TIMEOUT ) { printf( "INFO    : mode timeout = motor disabled because communication timeout\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_DISABLE_MOTOR_POWER ) { printf( "INFO    : mode disabled = motor disabled (after reset / or Linux software initialization)\n" ); }
			else if( rxPacket.pl.u8[0] == MODE_ZERO_CURRENT_CALIBRATION ) { printf( "INFO    : mode calibration = motor disabled for zero current calibration of drv8301\n" ); }
			else { printf( "ERROR   : no mode %d set, abort\n", rxPacket.pl.u8[0] ); exit(1); }
		}
	} else if ( rxPacket.responseType == RESP_ANGLE_TACHO_ZERO ) {
		if( rxPacket.payloadSize != 4 ) {
			printf( "ERROR   : invalid payload size for angle tacho zero value %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			angleZero = rxPacket.pl.u16[0];
			tachoZero = rxPacket.pl.u16[1];
			printf( "INFO    : angle zero %d, tacho zero %d\n", angleZero, tachoZero );
		}
	} else if ( rxPacket.responseType == RESP_DRV8301 ) {
		if( rxPacket.payloadSize != 12 ) {
			printf( "ERROR   : invalid payload size for the drv8301 %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			uint16_t status1 = rxPacket.pl.u16[0];
			uint16_t status2 = rxPacket.pl.u16[1];
			uint16_t control1 = rxPacket.pl.u16[2];
			uint16_t control1Strip  = 0x07ff & control1; // control value read from drv8301, but without the address
			uint16_t control2 = rxPacket.pl.u16[3];
			uint16_t control2Strip  = 0x07ff & control2; // control value read from drv8301, but without the address
			uint16_t setControl1 = 0x07fb & rxPacket.pl.u16[4]; // the b in 0x07fb because that value get's reset to zero after write
			uint16_t setControl2 = 0x07ff & rxPacket.pl.u16[5];
			if( control1Strip != setControl1 ) { printf( "ERROR   : drv8301 control1 0x%04x differs from set control1 0x%04x\n", control1Strip, setControl1 ); }
			if( control2Strip != setControl2 ) { printf( "ERROR   : drv8301 control2 0x%04x differs from set control2 0x%04x\n", control2Strip, setControl2 ); }

			printf( "INFO    : drv8301 status  1 (0x%04x):", status1 );
			if( ((status1>>0) & 1) == 1 ) { printf( ", FET low c over current (FETLC_OC)" ); }
			if( ((status1>>1) & 1) == 1 ) { printf( ", FET high c over current (FETHC_OC)" ); }
			if( ((status1>>2) & 1) == 1 ) { printf( ", FET low b over current (FETLB_OC)" ); }
			if( ((status1>>3) & 1) == 1 ) { printf( ", FET high b over current (FETHB_OC)" ); }
			if( ((status1>>4) & 1) == 1 ) { printf( ", FET low a over current (FETLA_OC)" ); }
			if( ((status1>>5) & 1) == 1 ) { printf( ", FET high a over current (FETHA_OC)" ); }
			if( ((status1>>6) & 1) == 1 ) { printf( ", junction overtemperature warning (OTW)" ); }
			if( ((status1>>7) & 1) == 1 ) { printf( ", junction overtemperature shutdown (OTSD)" ); }
			if( ((status1>>8) & 1) == 1 ) { printf( ", power supply undervoltage protection (PVDD_UV)" ); }
			if( ((status1>>9) & 1) == 1 ) { printf( ", internal gate driver undervoltage protection (GVDD_UV)" ); }
			if( ((status1>>10) & 1) == 1 ) { printf( ", shutdown occurred (FAULT)" ); }
			if( ((status1>>11) & 3) != 0 ) { printf( ", WRONG INDEX SHALL NEVEN HAPPEN!!" ); }
			if( status1  == 0 ) { printf( " no errors" ); }
			printf( "\n" );
			printf( "INFO    : drv8301 status  2 (0x%04x):", status2 );
			if( (status2 & 0xf) == 1 ) { printf( " device ID %d", status2 & 0xf ); } else { printf( " ERROR   : wrong device ID %d", status2 & 0xf ); }
			if( ((status2>>7) & 1) == 1 ) { printf( ", overvoltage protection limit (GVDD_OV)" ); }
			if( ((status2>>11) & 3) != 1 ) { printf( ", WRONG INDEX SHALL NEVEN HAPPEN!!" ); }
			printf( "\n" );
			printf( "INFO    : drv8301 control 1 (0x%04x):", control1 );
			if( ((control1>>0) & 3) == 0 ) {
				printf( " gate 1.7A" );
			} else if( ((control1>>0) & 3) == 1 ) {
				printf( " gate 0.7A" );
			} else if( ((control1>>0) & 3) == 2 ) {
				printf( " gate 0.25A" );
			} else {
				printf( " WRONG gate drive current SHALL NEVER HAPPEN" );
			}
			if( ((control1>>2) & 1) == 1 ) { printf( ", reset gate driver latched faults (reverts to 0) SHALL NEVER HAPPEN" ); }
			if( ((control1>>3) & 1) == 1 ) { printf( ", 3 PWM mode" ); } else { printf (", 6 PWM mode" ); }

			if( ((control1>>4) & 3) == 0 ) { printf( ", overcurrent protection mode current limit" ); }
			else if( ((control1>>4) & 3) == 1 ) { printf( ", OCP mode OC last shut down" ); }
			else if( ((control1>>4) & 3) == 2 ) { printf( ", OCP mode report only" ); }
			else { printf( ", OCP mode OC disable" ); }

			if( ((control1>>6) & 0x1f) == 0 ) { printf( ", OC Vds 0.060V" ); }
			else if( ((control1>>6) & 0x1f) == 1 ) { printf( ", OC Vds 0.068V" );	}
			else if( ((control1>>6) & 0x1f) == 2 ) { printf( ", OC Vds 0.076V" ); }
			else if( ((control1>>6) & 0x1f) == 3 ) { printf( ", OC Vds 0.086V" );	}
			else if( ((control1>>6) & 0x1f) == 4 ) { printf( ", OC Vds 0.097V" ); }
			else if( ((control1>>6) & 0x1f) == 5 ) { printf( ", OC Vds 0.109V" ); }
			else if( ((control1>>6) & 0x1f) == 6 ) { printf( ", OC Vds 0.123V" ); }
			else if( ((control1>>6) & 0x1f) == 7 ) { printf( ", OC Vds 0.138V" ); }
			else if( ((control1>>6) & 0x1f) == 8 ) { printf( ", OC Vds 0.155V" ); }
			else if( ((control1>>6) & 0x1f) == 9 ) { printf( ", OC Vds 0.175V" ); }
			else if( ((control1>>6) & 0x1f) == 10 ) { printf( ", OC Vds 0.197V" ); }
			else if( ((control1>>6) & 0x1f) == 11 ) { printf( ", OC Vds 0.222V" ); }
			else if( ((control1>>6) & 0x1f) == 12 ) { printf( ", OC Vds 0.250V" ); }
			else if( ((control1>>6) & 0x1f) == 13 ) { printf( ", OC Vds 0.282V" ); }
			else if( ((control1>>6) & 0x1f) == 14 ) { printf( ", OC Vds 0.317V" ); }
			else if( ((control1>>6) & 0x1f) == 15 ) { printf( ", OC Vds 0.358V" ); }
			else if( ((control1>>6) & 0x1f) == 16 ) { printf( ", OC Vds 0.403V" ); } // this is the drv8301 power-up default
			else if( ((control1>>6) & 0x1f) == 17 ) { printf( ", OC Vds 0.454V" ); }
			else if( ((control1>>6) & 0x1f) == 18 ) { printf( ", OC Vds 0.511V" ); }
			else if( ((control1>>6) & 0x1f) == 19 ) { printf( ", OC Vds 0.576V" ); }
			else if( ((control1>>6) & 0x1f) == 20 ) { printf( ", OC Vds 0.648V" ); }
			else if( ((control1>>6) & 0x1f) == 21 ) { printf( ", OC Vds 0.730V" ); }
			else if( ((control1>>6) & 0x1f) == 22 ) { printf( ", OC Vds 0.822V" ); }
			else if( ((control1>>6) & 0x1f) == 23 ) { printf( ", OC Vds 0.926V" ); }
			else if( ((control1>>6) & 0x1f) == 24 ) { printf( ", OC Vds 1.043V" ); }
			else if( ((control1>>6) & 0x1f) == 25 ) { printf( ", OC Vds 1.175V" ); }
			else if( ((control1>>6) & 0x1f) == 26 ) { printf( ", OC Vds 1.324V" ); }
			else if( ((control1>>6) & 0x1f) == 27 ) { printf( ", OC Vds 1.491V" ); }
			else if( ((control1>>6) & 0x1f) == 28 ) { printf( ", OC Vds 1.679V" ); }
			else if( ((control1>>6) & 0x1f) == 29 ) { printf( ", OC Vds 1.892V" ); }
			else if( ((control1>>6) & 0x1f) == 30 ) { printf( ", OC Vds 2.131V" ); }
			else { printf( ", OC Vds 2.400V" ); }

			if( ((control1>>11) & 3) != 2 ) { printf( ", WRONG INDEX SHALL NEVEN HAPPEN!!" ); }
			printf( "\n" );

			printf( "INFO    : drv8301 control 2 (0x%04x):", control2 );

			if( ((control2>>0) & 3) == 0 ) { printf( " report OT and OC" ); }
			else if( ((control2>>0) & 3) == 1 ) { printf( " report OT only" ); }
			else if( ((control2>>0) & 3) == 2 ) { printf( " report OC only" ); }
			else { printf( " report reserved SHALL NEVER HAPPEN" ); }

			if( ((control2>>2) & 3) == 0 ) { printf( ", shunt 10V/V" ); }
			else if( ((control2>>2) & 3) == 1 ) { printf( ", shunt 20V/V" ); }
			else if( ((control2>>2) & 3) == 2 ) { printf( ", shunt 40V/V" ); }
			else { printf( ", shunt 80V/V" ); }

			if( ((control2>>4) & 1) == 1 ) { printf( ", external calibration channel 1" ); }
			if( ((control2>>5) & 1) == 1 ) { printf( ", external calibration channel 2" ); }
			if( ((control2>>6) & 1) == 1 ) { printf( ", OC_TOFF off-time control" ); } else { printf( ", OC_TOFF cycle by cycle" ); }
			if( ((control2>>11) & 3) != 3 ) { printf( ", WRONG INDEX SHALL NEVEN HAPPEN!!"); }
			printf( "\n" );
		}
	} else if ( rxPacket.responseType == RESP_AA_BOARD ) {
		if( rxPacket.payloadSize != 24 ) {
			printf( "ERROR   : invalid payload size for board response %d\n", rxPacket.payloadSize ); exit(1);
		} else {
			printf("Device ID: 0x%04x\n", rxPacket.pl.u16[0]);
			printf("Vendor ID: 0x%04x\n", rxPacket.pl.u16[1]);
			printf("Application ID : 0x%04x\n", rxPacket.pl.u16[2]);
			printf("Hardware Version : 0x%04x\n", rxPacket.pl.u16[3]);
			printf("Software Version : 0x%04x\n", rxPacket.pl.u16[4]);
			// printf("Placeholder : 0x%04x\n", rxPacket.pl.u16[5]);
			printf("Xmega Device ID : 0x%08x\n", rxPacket.pl.u32[3]);
			printf("Xmega Serial : 0x%08x\n", rxPacket.pl.u32[4]);
			printf("CPU Clock: %d Hz\n", rxPacket.pl.u32[5]);

			if( rxPacket.pl.u16[3] != HARDWARE_VERSION ) {
				printf( "ERROR   : invalid different hardware version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", HARDWARE_VERSION, HARDWARE_VERSION, rxPacket.pl.u16[3], rxPacket.pl.u16[3] ); exit(1);
			}
			if( rxPacket.pl.u16[4] != SOFTWARE_VERSION ) {
				printf( "ERROR   : invalid different software version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", SOFTWARE_VERSION, SOFTWARE_VERSION, rxPacket.pl.u16[4], rxPacket.pl.u16[4] ); exit(1);
			}
			if( applicationId == 255 ) {
				// applicationId was not set, then it should be a value from 1 to 6
				if( rxPacket.pl.u16[2] == 0 ) {
					printf( "ERROR   : invalid Application ID %d, probably short, abort\n", rxPacket.pl.u16[2] ); exit(1);
				}
				if( rxPacket.pl.u16[2] == 7 ) {
					printf( "ERROR   : invalid Application ID %d, probably open, abort\n", rxPacket.pl.u16[2] ); exit(1);
				}
			} else {
				if( rxPacket.pl.u16[2] != applicationId ) {
					printf( "ERROR   : invalid Application ID %d, expected %d, abort\n", rxPacket.pl.u16[2], applicationId ); exit(1);
				}
			}
		}
	} else {
		printf( "receive : %3d %3d %3d %3d | ", receiveAckIdMis, receiveChecksumFail, packetSize, farEndBufferSize );
		char *rxPacketPointer = (char *) &rxPacket; // use char pointer to index the individual bytes of the rxPacket
		for( int ii = 0; ii < packetSize; ii++ ) {
			printf( "%02x ", (uint8_t) *rxPacketPointer );
 			if( ii == (FROM_BOARD_PACKET_HEADER_SIZE-1) ) { printf( "| " ); }
			rxPacketPointer++;
		}
 		printf( "\n" );
	}
}

void printRemoteErrors( ) {
	uint16_t commError = rxPacket.pl.u16[0];
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_WRITE) != 0 ) { printf( "ERROR   : far end communication from board write!\n"); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : far end communication from board out of range!\n"); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_END_CHECK) != 0 ) { printf( "ERROR   : far end communication from board end check!\n"); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN) != 0 ) { printf( "ERROR   : far end communication from board sof overwritten!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW) != 0 ) { printf( "ERROR   : far end communication to board size overflow!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_OVERFLOW) != 0 ) { printf( "ERROR   : far end communication to board overflow!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_CHECKSUM) != 0 ) { printf( "ERROR   : far end communication to board checksum!\n"); farEndChecksumFail++; }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_END_CHECK) != 0 ) { printf( "ERROR   : far end communication to board end check!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT) != 0 ) { printf( "ERROR   : far end communication to board sof alignment!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND) != 0 ) { printf( "ERROR   : far end communication to board illegal command!\n"); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC) != 0 ) { printf( "ERROR   : far end communication to board ack id out of sync!\n"); farEndAckIdMis++; }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN) != 0 ) { printf( "ERROR   : far end communication to board packet overwritten!\n"); farEndAckIdMis++; }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW) != 0 ) { printf( "ERROR   : far end communication to board buffer overflow!\n"); farEndAckIdMis++; }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE) != 0 ) { printf( "ERROR   : far end communication to board incorrect amount of bytes received!\n"); farEndAckIdMis++; }

	uint16_t encErrror = rxPacket.pl.u16[1];
	if( ( encErrror & ENCODER_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : encoder not initialized!\n"); }
	if( ( encErrror & ENCODER_ERROR_VELOCITY_MAX_NEGATIVE) != 0 ) { printf( "ERROR   : encoder maximal negative motor speed!\n"); }
	if( ( encErrror & ENCODER_ERROR_VELOCITY_MAX_POSITIVE) != 0 ) { printf( "ERROR   : encoder maximal positive motor speed!\n"); }

	uint16_t pidError = rxPacket.pl.u16[2];
	if( ( pidError & PID_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : pid not initialized!\n"); }
	if( ( pidError & PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : primary pid error value out of range!\n"); }
	if( ( pidError & PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : primary pid integral out of range!\n"); }
	if( ( pidError & PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : primary pid derivative out of range!\n"); }
	if( ( pidError & PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : primary pid result out of range!\n"); }
	if( ( pidError & PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : angle pid error value out of range!\n"); }
	if( ( pidError & PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : angle pid integral out of range!\n"); }
	if( ( pidError & PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : angle pid derivative out of range!\n"); }
	if( ( pidError & PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE) != 0 ) { printf( "ERROR   : angle pid result out of range!\n"); }

	uint16_t pwmError = rxPacket.pl.u16[3];
	if( ( pwmError & PWM_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : pwm not initialized!\n"); }
	if( ( pwmError & PWM_ERROR_POSITIVE_LIMIT) != 0 ) { printf( "ERROR   : pwm error positive limit!\n"); }
	if( ( pwmError & PWM_ERROR_NEGATIVE_LIMIT) != 0 ) { printf( "ERROR   : pwm error negative limit!\n"); }
	if( ( pwmError & PWM_ERROR_POSITIVE_DELTA) != 0 ) { printf( "ERROR   : pwm error positive delta (increase to fast)!\n"); }
	if( ( pwmError & PWM_ERROR_NEGATIVE_DELTA) != 0 ) { printf( "ERROR   : pwm error negative delta (reverse to fast)!\n"); }

	uint16_t safetyError = rxPacket.pl.u16[4];
	if( ( safetyError & SAFETY_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : safety not initialized!\n"); }
	if( ( safetyError & SAFETY_COMMUNICATION_TIMEOUT) != 0 ) { printf( "ERROR   : safety says board did not receive new packet in time! (motorTimeout)\n"); }
	if( ( safetyError & SAFETY_ANGLE_TACHO_UNINITIALZED) != 0 ) { printf( "ERROR   : safety says un-initialized angle and or tacho used!, abort\n"); exit(1); }
	// the next error might appear when noise on the hall sensor
	if( ( safetyError & SAFETY_ANGLE_VALUE_TOO_HIGH) != 0 ) { printf( "ERROR   : safety says angle value too high!, probaly unconnected hall sensor\n"); }
	if( ( safetyError & SAFETY_ANGLE_VALUE_TOO_LOW) != 0 ) { printf( "ERROR   : safety says angle value too low!, probably short on hall sensor\n"); }
	if( ( safetyError & SAFETY_TACHO_VALUE_TOO_HIGH) != 0 ) { printf( "ERROR   : safety says tacho value too high!, abort\n"); exit(1); }
	if( ( safetyError & SAFETY_TACHO_VALUE_TOO_LOW) != 0 ) { printf( "ERROR   : safety says tacho value too low!, abort\n"); exit(1); }

	uint16_t schedulerError = rxPacket.pl.u16[5];
	if( ( schedulerError & SCHEDULER_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : scheduler not initialized!\n"); }
	if( ( schedulerError & SCHEDULER_ERROR_MEASURE_TIME_EXPIRED) != 0 ) { printf( "ERROR   : scheduler measurement cycle to long!\n"); }
	if( ( schedulerError & SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED) != 0 ) { printf( "ERROR   : scheduler calculation cycle to long!\n"); }
	if( ( schedulerError & SCHEDULER_ERROR_TEST) != 0 ) { printf( "ERROR   : scheduler test\n"); }

	uint16_t drv8301Error = rxPacket.pl.u16[6];
	if( ( drv8301Error & DRV8301_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "ERROR   : drv8301 not initialized!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETLC_OC) != 0 ) { printf( "ERROR   : drv8301 error FET low c over current (FETLC_OC) !\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETHC_OC) != 0 ) { printf( "ERROR   : drv8301 error FET high c over current (FETHC_OC)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETLB_OC) != 0 ) { printf( "ERROR   : drv8301 error FET low b over current (FETLB_OC)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETHB_OC) != 0 ) { printf( "ERROR   : drv8301 error FET high b over current (FETHB_OC)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETLA_OC) != 0 ) { printf( "ERROR   : drv8301 error FET low a over current (FETLA_OC)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FETHA_OC) != 0 ) { printf( "ERROR   : drv8301 error FET high a over current (FETHA_OC)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_OTW) != 0 ) { printf( "ERROR   : drv8301 error junction overtemperature warning (OTW)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_OTSD) != 0 ) { printf( "ERROR   : drv8301 error junction overtemperature shutdown (OTSD)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_PVDD_UV) != 0 ) { printf( "ERROR   : drv8301 error power supply undervoltage protection (PVDD_UV)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_GVDD_UV) != 0 ) { printf( "ERROR   : drv8301 error internal gate driver undervoltage protection (GVDD_UV)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_FAULT) != 0 ) { printf( "ERROR   : drv8301 error shutdown occurred (FAULT)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_DEVICE_ID) != 0 ) { printf( "ERROR   : drv8301 error wrong Device ID!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_GVDD_OV) != 0 ) { printf( "ERROR   : drv8301 error overvoltage protection limit (GVDD_OV)!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_CONTROL_REGISTER1) != 0 ) { printf( "ERROR   : drv8301 write to control register 1 unsuccessful!\n"); }
	if( ( drv8301Error & DRV8301_ERROR_CONTROL_REGISTER2) != 0 ) { printf( "ERROR   : drv8301 write to control register 2 unsuccessful!\n"); }
}

void sendClearAllErrors( ) {
	txPacket.command = CMD_CLEAR_ERRORS;
	txPacket.payloadSize = 14;
	for( int ii = 0; ii < txPacket.payloadSize; ii++ ){
		txPacket.pl.u8[ii] = 0xff;
	}
	sendPacket( );
}

void sendClearKnownErrors( ) {
	txPacket.command = CMD_CLEAR_ERRORS;
	txPacket.payloadSize = 14;
	for( int ii = 0; ii < 7; ii++ ){
		txPacket.pl.u16[ii] = rxPacket.pl.u16[ii];
	}
	sendPacket( );
}

// print received error package and send message to clear all remote errors
void clearRemoteErrors( ) {
	printf("\nWARNING : received the following error message from far end\n");
	printReceivedPacket();
	printRemoteErrors();
	sendClearKnownErrors();
}

// Check if calculated packet checksum is the same as the checksum provided by the far end
void rxPacketAckIdCheck() {
	static uint8_t ackId = 0;
	static int initialized = 0;
	if( initialized > 2 ) {
		// sometimes 2 packets are missed before in sync, maybe there is an overwrite in the receive buffer at startup
		if( ackId != rxPacket.ackId ) {
			printf("ERROR   : packet ack id is 0x%02x but should be 0x%02x\n", rxPacket.ackId, ackId);
			receiveAckIdMis++;
		}
	} else {
		initialized++;
	}
	ackId = rxPacket.ackId + 1;
}

void rxPacketChecksumCheck() {
	uint8_t checksum = 0;
	int ii = 0;
	char *rxPacketPointer = (char *) &rxPacket; // use char pointer to index the individual bytes of the rxPacket
	for( ii = 0; ii < ( FROM_BOARD_PACKET_HEADER_SIZE + rxPacket.payloadSize ); ii++ ) {
		checksum += *rxPacketPointer;
		rxPacketPointer++;
	}
	checksum -= rxPacket.checksum; // checksum added in above loop, but checksum itself should be excluded from check calculation

	if( checksum != rxPacket.checksum ) {
		printf("ERROR   : checksum error on this received package:\n");
		printReceivedPacket();
		printf("ERROR   : packet checksum is 0x%02x but calculated checksum is 0x%02x\n", rxPacket.checksum, checksum);
		receiveChecksumFail++;
	}
}

// process the received package
void processReceivedPacket( ) {
	receivedPacketsCnt++;
	rxPacketAckIdCheck();
	rxPacketChecksumCheck();
	farEndBufferSize = rxPacket.bufferSpace;
	// the sending and receiving is asynchronous
	// worst case the total amount of send bytes from the previous received packet should fit
	// inside the last reported far end buffer size
	bytesSendDuringMin2farEndBufferSizeRefresh = bytesSendDuringMin1farEndBufferSizeRefresh;
	bytesSendDuringMin1farEndBufferSizeRefresh = bytesSendDuringCurrentfarEndBufferSizeRefresh;
	bytesSendDuringCurrentfarEndBufferSizeRefresh = 0;
	if( rxPacket.responseType == RESP_ERROR ) {
		clearRemoteErrors( );
	} else {
		printReceivedPacket();
	}
}

// Receive a number of packets from the serial input buffer
// The maximal size per read is the maximal packet size
// However most packets will be smaller then the maximal packet size, so probably
// multiple packets will be collected with a single read to the serial port
// This function blocks until data is available on the serial port
// If data is received it first start searching for the start of frame (= begin of packet)
// Then it will continue with adding bytes to the packet until the amount bytes is the same as the
// packet size (header + payload size) provided from the far end (board).
// When the packet is complete the processReceivedPacket function will be called.
// If there was more data read then for the packet, a search for the next start of frame will start
// to assemble the next packet.
void receiveBytes( ) {
	// TODO: packet rxPacketTmp should be local to this function, however then a initialization solution has to be found
	static int state = 0; // state 0 = search SOF, state 1 = complete the packet
	static int rxBytes = 0; // current packet byte size
	static char *rxPacketTmpPointer = 0; // for efficient iterating through the rx packet

	// get a number of bytes from the input buffer
	unsigned char rxBuffer[1024];
	int numBytes = read(port, &rxBuffer, FROM_BOARD_PACKET_SIZE);

	if( numBytes < 0 ) { perror("Error   : reading data"); }

	// process byte for byte, first search for the sof, then complete the packet
	for( int ii = 0; ii < numBytes; ii++ ) {

		switch (state){
		case 0: // state: search SOF
			if( rxBuffer[ii] == SOF ){
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
			if( rxBytes <= FROM_BOARD_PACKET_SIZE ) {
				*rxPacketTmpPointer = rxBuffer[ii]; // store received byte in rx packet
				rxPacketTmpPointer++; // move pointer to the next location of the rx packet (to store the next byte)
				if( rxBytes >= 4 ) { // TODO: replace by e.g. (rxPacketTmp.payloadSize-rxPacketTmp.sof)
					// size of payload (provided by far end) is available
					if( rxBytes == ((uint16_t)rxPacketTmp.payloadSize) + FROM_BOARD_PACKET_HEADER_SIZE ) {
						// received the last byte of current packet, packet complete, all done
						memcpy((void *) &rxPacket, (void *) &rxPacketTmp, rxPacketTmp.payloadSize + FROM_BOARD_PACKET_HEADER_SIZE );
						processReceivedPacket( );
						state = 0; // start searching for the next packet
						if( rxPacketTmp.endCheck != 0x98 ) { // check if rx buffer was written out of range
							printf( "ERROR   : overwritten end of rx packet\n");
							exit(1);
						}
					} else if( rxBytes > ((uint16_t)rxPacketTmp.payloadSize) + FROM_BOARD_PACKET_HEADER_SIZE ) {
						printf("ERROR   : more bytes received then set by the far end through the payload size\n");
						exit(1);
					}
				}
			} else {
				printf("ERROR   : rx packet (buffer) full, should never happen\n");
				exit(1);
			}
			break;
		default:
			break;
		}
	} // for ii < numBytes
}

void setLedYellow( ) {
	txPacket.command = CMD_SET_LED_YELLOW;
	txPacket.payloadSize = 1;
	if( ledYellow ) { txPacket.pl.u8[0] = 0; } else { txPacket.pl.u8[0] = 1; } // toggle earlier read value
	sendPacket( );
}

void getLedYellow( ) {
	txPacket.command = CMD_GET_LED_YELLOW;
	txPacket.payloadSize = 0;
	sendPacket( );
	// actual value of the led is available in one of the next responses of the board
}

void setSetPoint( uint8_t command, int value ) {
	if( value < -32768 || value > 32767 ) { printf("Error   : velocity set point out of range %d\n", value ); exit(1); }
	txPacket.command = command;
	txPacket.payloadSize = 2;
	txPacket.pl.s16[0] = value;
	sendPacket( );
}

void setPidProperties( uint8_t command, int p, int i, int d, int iTh, int iMax ) {
	if( p < 0 || p > 0x7fff ) { printf("Error   : pid p out of range %d\n", p ); exit(1); }
	if( i < 0 || i > 0x7fff ) { printf("Error   : pid i out of range %d\n", i ); exit(1); }
	if( d < 0 || d > 0x7fff ) { printf("Error   : pid d out of range %d\n", d ); exit(1); }
	if( iTh < 0 || iTh > 0x7fff ) { printf("Error   : pid iTh out of range %d\n", iTh ); exit(1); }
	if( iMax < 0 || iMax > 0x7fffffff ) { printf("Error   : pid iMax out of range %d\n", iMax ); exit(1); }
	txPacket.command = command;
	txPacket.payloadSize = 12;
	txPacket.pl.u16[0] = p;
	txPacket.pl.u16[1] = i;
	txPacket.pl.u16[2] = d;
	txPacket.pl.u16[3] = iTh;
	txPacket.pl.u32[2] = iMax;
	sendPacket( );
}

void setMotorTimeout( int value ) {
	txPacket.command = CMD_SET_MOTOR_TIMEOUT;
	txPacket.payloadSize = 2;
	txPacket.pl.u16[0] = value;
	sendPacket( );
}

void setPwmLimit( int value ) {
	txPacket.command = CMD_SET_PWM_LIMIT;
	txPacket.payloadSize = 2;
	txPacket.pl.u16[0] = value;
	sendPacket( );
}

void setPwmDelta( int value ) {
	txPacket.command = CMD_SET_PWM_DELTA;
	txPacket.payloadSize = 4;
	txPacket.pl.u32[0] = value;
	sendPacket( );
}

void setDrv8301( uint16_t control1, uint16_t control2 ) {
	txPacket.command = CMD_SET_DRV8301;
	txPacket.payloadSize = 4;
	txPacket.pl.u16[0] = control1;
	txPacket.pl.u16[1] = control2;
	sendPacket( );
}

void setMode( uint8_t mode ) {
	txPacket.command = CMD_SET_MODE;
	txPacket.pl.u8[0] = mode;
	txPacket.payloadSize = 1;
	sendPacket( );
}

void setAngleDirection( bool value ) {
	txPacket.command = CMD_SET_ANGLE_DIRECTION;
	txPacket.pl.u8[0] = (uint8_t) value;
	txPacket.payloadSize = 1;
	sendPacket( );
}

void getCommand( uint8_t command ) {
	txPacket.command = command;
	txPacket.payloadSize = 0;
	sendPacket( );
	// actual return data is available in one of the next responses of the board
}

int main(int argc, char** argv) {
	int opt = 0;
	_Bool quiet = 0;
	char portName[256] = "/dev/ttyUSB0";
	while( (opt = getopt(argc, argv, "a:hP:q") ) != -1 ) {
		switch(opt) {
		case 'a':
			applicationId = atoi(optarg);
			break;
		case 'h':
			printf("options:\n");
			printf("  -P serial port e.g. /dev/ttyUSB0\n");
			printf("  -q quiet\n");
			printf("  -h this help\n");
			return 0;
			break;
		case 'P':
			strcpy(portName, optarg);
			break;
		case 'q':
			quiet = 1;
			break;
		}
	}

	if( ! quiet ) { printf("test application to control the peripheral boards\n"); }

	struct termios tio;

	memset(&tio, 0, sizeof(tio));
	tio.c_iflag = 0;
	tio.c_oflag = 0;
	tio.c_cflag = CS8 | CREAD | CLOCAL; // 8n1, see termios.h for more information
	tio.c_lflag = 0;
	tio.c_cc[VMIN] = 0; // Minimum number of characters for noncanonical read (MIN).
	tio.c_cc[VTIME] = 5; // Timeout in deciseconds for noncanonical read (TIME) 5 = 500ms //

	if( ! quiet ) {	printf("using serial interface: %s\n", portName); }
	port = open(portName, O_RDWR ); // use global variable (in the C++ this will be performed in the constructor)

	// do not use O_NONBLOCK, because we want to wait for a reply until the VTIME expires
	// this prevents adding a nanosleep between the write and read
    // nanosleep((struct timespec[]){{0, 10*1000L}}, NULL); /* 1000 us */
	// O_NONBLOCK is posix-specified name for O_NDELAY
	// O_NOCTTY If set and path identifies a terminal device, open() shall not cause the terminal device to become the controlling terminal for the process.
	cfsetospeed(&tio, B115200);
	cfsetispeed(&tio, B115200);

	tcsetattr(port, TCSANOW, &tio);

	rxPacketTmp.sof = SOF; // should never be overwritten
	rxPacketTmp.ackId = 0xff;
	rxPacketTmp.feedbackId = 0xff;
	rxPacketTmp.responseType = 0xff;
	rxPacketTmp.payloadSize = 0;
	rxPacketTmp.checksum = 0xff; // is overwritten when receiving packet, but never read
	rxPacketTmp.endCheck = 0x98; // should never be overwritten

	printf("wait until far end receive buffer is empty");
	while( farEndBufferSize < 255 ) { // wait until far end is able to receive data (the far end receive buffer should be empty)
		receiveBytes( );
		printf(".");
	}
	printf("\n");

	sendClearAllErrors( ); // reset far end to prevent e.g. ack id errors when restarting linux application, also reset pid integral value
	getCommand( CMD_AA_GET_BOARD );
	setMode( MODE_DISABLE_MOTOR_POWER );
	getCommand( CMD_GET_MODE );
	setMotorTimeout( 200 ); // 2.5ms * 200 = 500ms, maximal 2.5ms * 65535 = 163.8 seconds
	getCommand( CMD_GET_MOTOR_TIMEOUT );
	// setPwmDelta( 1<<14 ); // 0.5/648 = 0.077% per 2.5ms = 30% per second, full speed (100% = 648) in 3.3 seconds
	// setPwmDelta( 200<<16 ); //
	setPwmDelta( 147704 ); //
	getCommand( CMD_GET_PWM_DELTA );
	setPwmLimit( 600 ); // 648 = 100%
	getCommand( CMD_GET_PWM_LIMIT );
	// wheel motor kp > 18000 results in oscillation on free running motor
	setPidProperties( CMD_SET_PID_PRIMARY_PROPERTIES, 30000, 700, 0, 2, 30208 ); // p, i, d, iTh, iMax
	getCommand( CMD_GET_PID_PRIMARY_PROPERTIES );
	setPidProperties( CMD_SET_PID_ANGLE_PROPERTIES, 4000, 0, 0, 0, 100000 ); // p, i, d, iTh, iMax, there is no feedback in the angle, so the i does not make sense
	getCommand( CMD_GET_PID_ANGLE_PROPERTIES );
	// setDrv8301( 0x288, (0x00 | (3<<2)) ); // 80V/V
	// setDrv8301( (0x288 | (1<<2) ), 0x00 ); // reset gate driver
	// setDrv8301( 0x288, 0x00 );
	uint16_t control1 = 0;
	// control1 |=  0<<0; // GATE_CURRENT : Gate drive peak current 1.7 A, gives a spike of 750mV peak on shunt resistor
	// control1 |=  1<<0; // GATE_CURRENT : Gate drive peak current 0.7 A
	control1 |=  1<<1; // GATE_CURRENT : Gate drive peak current 0.25 A, gives a spike of 200mV peak on shunt resistor
	control1 |=  1<<2; // GATE_RESET : Reset gate driver latched faults (reverts to 0) (required if you got the drv8301 error shutdown occurred (FAULT))
	control1 |=  1<<3; // PWM_MODE : 3 PWM inputs
	control1 |=  0<<4; // OCP_MODE : Current limit (works together with OC_ADJ_SET)
	// control1 |=  1<<4; // OCP_MODE : OC latch shut down (requires a gate reset to resolve)
	// OC_ADJ_SET VDS = iOverCurrent x rdsOnFet (irf7749) => 60mV / 1.1mOhm = 54.5A => does not work, gives quickly errors when changing direction
	// use current limit instead of shutdown
	// control1 |= 0<<6; // OC_ADJ_SET : Over current Adjustment = 60mV
	control1 |= 10<<6; // OC_ADJ_SET : Over current Adjustment = 197mV
	// control1 |= 31<<6; // OC_ADJ_SET : Over current Adjustment = 2.4V
	uint16_t control2 = 0;
	control2 |= 0<<0; // OCTW_MODE : Report both over temperature (OT) and over current (OC) at nOCTW pin
	control2 |= 2<<2; // GAIN : Gain of shunt amplifier: 40 V/V, with 30A ADC range between 0.05V to 2.45V, so best tradeoff between high current and noise
	// control2 |= 1<<4; // DC_CAL_CH1 : calibrate shunt amplifier 1
	// control2 |= 1<<5; // DC_CAL_CH2 : calibrate shunt amplifier 2
	control2 |= 0<<6; // OC_TOFF : Cycle by cycle = the MOSFET on which over current has been detected on will shut off until the next PWM cycle
	setDrv8301( control1, control2 );
	getCommand( CMD_GET_DRV8301);

	int ii = 0;
	int jj = 0;
	_Bool increment = 1;

	// first perform the current calibration, reading the current a and current b, while using a setpoint of 0
	currentCalibrate = true;
	setMode( MODE_ZERO_CURRENT_CALIBRATION );
	getCommand( CMD_GET_MODE );
	while( currentCalibrate ) {
		receiveBytes( );
		if( (farEndBufferSize - bytesSendDuringCurrentfarEndBufferSizeRefresh - bytesSendDuringMin1farEndBufferSizeRefresh - bytesSendDuringMin2farEndBufferSizeRefresh > 127 ) ) {
			setSetPoint( CMD_SET_PRIMARY_SETPOINT, 0 ); // to prevent motorTimeout, value is not used
		}
	}

	// setMode( MODE_PID_TACHO ); // setMode also sets the motor speed to zero (no spinning ballHandler is a setPoint around 2330)
	setAngleDirection( true ); // invert the angle behavior between left and right ball handler (clockwise/anti-clockwise behavior increase/decrease)
	// setMode( MODE_PID_ANGLE ); // this is the mode for ball handler motors, setMode also sets the motor speed to zero (no spinning ballHandler is a setPoint around 2330)
	// setMode( MODE_PID_ENCODER ); // this is the mode for wheel motors
	setMode( MODE_PWM_ONLY ); // no magic, just provide this amount of power to the motors
	getCommand( CMD_GET_MODE ); // do not disable the get mode because the return value is used for the functionality
	getCommand( CMD_GET_ANGLE_TACHO_ZERO );
	getCommand( CMD_GET_ANGLE_DIRECTION );

	while( 1 ) {
		receiveBytes( );
		if( ii == 1000 ) { // when sending two or more large packets 2*(header 5 + payload 255) without some time in between the rx buffer (255 bytes) on the far end will overflow
			if ( increment ) { jj+=50;} else { jj = jj; } // jj-=5; }
			if( jj > 500) { increment = false; }
			if( jj < -200 ) { increment = true; }

		} else if( ii == 1001 ) {
			ii = 0;
		} else if( ( ii % 10) == 5 ) {
			// we need to keep on sending data (could be set points) otherwise the safety taks will disable the motor
			if( (farEndBufferSize - bytesSendDuringCurrentfarEndBufferSizeRefresh - bytesSendDuringMin1farEndBufferSizeRefresh - bytesSendDuringMin2farEndBufferSizeRefresh > 127 ) ) {
				sendLoopBackPacket( );
			}
		}
		if( (farEndBufferSize - bytesSendDuringCurrentfarEndBufferSizeRefresh - bytesSendDuringMin1farEndBufferSizeRefresh - bytesSendDuringMin2farEndBufferSizeRefresh > 127 ) ) {
			if( mode == MODE_PWM_ONLY ) {
				setSetPoint( CMD_SET_PRIMARY_SETPOINT, jj );
			} else if( mode == MODE_PID_ENCODER ) {
				setSetPoint( CMD_SET_PRIMARY_SETPOINT, jj ); // 780 = 5.031 m/s
			} else if ( mode == MODE_PID_TACHO ) {
				if( tachoZero != 0 ) { // first wait until we know what the adc value is when the motor is not turning
					setSetPoint( CMD_SET_PRIMARY_SETPOINT, tachoZero + 1.5*jj );
				}
			} else if ( mode == MODE_PID_ANGLE ) {
				if( tachoZero != 0 ) { // first wait until we know what the adc value is when the motor is not turning
					setSetPoint( CMD_SET_PRIMARY_SETPOINT, tachoZero ); // not required, during mode change the primary setPoint is set to tachoZero on the board
					setSetPoint( CMD_SET_ANGLE_SETPOINT, angleZero + 0.8*jj );
				}
			} else {
				if( ii == 999 ) {
					printf( "INFO    : no mode set, abort\n"  );
					exit(1);
				}
			}
		}
		ii++;
	}

	close(port);
	return 0;
}
