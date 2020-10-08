 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 #include <math.h>


#include "motorBoard.hpp"

using namespace std;

motorBoard::motorBoard( ) {
}

void motorBoard::printSendPacket( string prefix, toBoardPacketT txPacket ) {
	if( txPacket.command == CMD_LOOPBACK ||
		txPacket.command == CMD_AA_GET_BOARD ||
		txPacket.command == CMD_CLEAR_ERRORS ||
		txPacket.command == CMD_GET_LED_GREEN ||
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
		txPacket.command == CMD_GET_PRIMARY_SETPOINT ||
		txPacket.command == CMD_GET_ANGLE_SETPOINT ||
		txPacket.command == CMD_SET_LED_YELLOW ) {
		// for these packets we do not need to inspect the transmitted packet
	} else if( txPacket.command == CMD_SET_ANGLE_SETPOINT ) {
 		// printf("send    : set angle setPoint to %5d\n", txPacket.pl.s16[0] );
	} else if( txPacket.command == CMD_SET_PRIMARY_SETPOINT ) {
 		// printf("send    : set wheel speed (or tacho offset) to %5d = %4.3f m/s\n", txPacket.pl.s16[0], txPacket.pl.s16[0]*oneWheelTick/oneTimeTick);
	} else if( txPacket.command == CMD_SET_MOTOR_TIMEOUT ) {
 		// printf("send    : set motor timeout %4.0f ms\n", txPacket.pl.u16[0] * 2.5);
	} else if( txPacket.command == CMD_SET_MODE ) {
		if( txPacket.pl.u8[0] == MODE_PID_ANGLE ) {	printf( "%s INFO    : send mode angle = first pid input is angle + setPoint, second pid input is first pid + tacho\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_PID_ENCODER ) { printf( "%s INFO    : send mode encoder = pid input is encoder + setPoint\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_PID_TACHO ) { printf( "%s INFO    : send mode tacho = pid input is tacho + setPoint\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_PWM_ONLY ) { printf( "%s INFO    : send mode pwm only = no pid, pwm is directly controlled from setPoint\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_COMMUNICATION_TIMEOUT ) { } // TODO enable again when print flooding fixed printf( "%s INFO    : send mode timeout = motor disabled because communication timeout\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_DISABLE_MOTOR_POWER ) { printf( "%s INFO    : send mode disable = motor disabled (after reset / or Linux software initialization)\n", prefix.c_str( ) ); }
		else if( txPacket.pl.u8[0] == MODE_ZERO_CURRENT_CALIBRATION ) { printf( "%s INFO    : send mode calibration = motor disabled for zero current calibration of drv8301\n", prefix.c_str( ) ); }
		else { printf( "%s ERROR   : no mode set %d, abort\n", prefix.c_str( ), txPacket.pl.u8[0] ); exit(1); }
	} else if( txPacket.command == CMD_SET_DRV8301 ) {
		uint16_t control1 = txPacket.pl.u16[0];
		printf( "%s INFO    : send drv8301 control 1 (0x%04x):", prefix.c_str( ), control1 );
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
		printf( "%s INFO    : send drv8301 control 2 (0x%04x):", prefix.c_str( ), control2 );
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
 		printf( "%s INFO    : send %3d | ", prefix.c_str( ), packetSize );
 		char *txPacketPointer = (char *) &txPacket; // use char pointer to index the individual bytes of the txPacket
 		for( int ii = 0; ii < packetSize; ii++) {
 			printf( "%02x ", (uint8_t) *txPacketPointer );
 			if( ii == (TO_BOARD_PACKET_HEADER_SIZE-1) ) { printf( "   | " ); }
 			txPacketPointer++;
 		}
 		printf("\n");
 	}
}

void motorBoard::printReceivedPacket( string prefix, fromBoardPacketT rxPacket, uint8_t mode, int16_t primarySetPoint, uint16_t applicationId ) {
	int packetSize = FROM_BOARD_PACKET_HEADER_SIZE + rxPacket.payloadSize;
	static int defaultRespCounter = 0; // we do not want to print all default responses
	static float currentAMeasure = 0; // average out current A to get rid of the noise
	static float currentBMeasure = 0; // average out current B to get rid of the noise
	static size_t currentMeasureCount = 0;

	if( rxPacket.responseType == RESP_LED_YELLOW ) {
		if( rxPacket.pl.u8[0] == 1 ) {
			printf( "%s INFO    : yellow led is on\n", prefix.c_str( ) );
		} else {
			printf( "%s INFO    : yellow led is off\n", prefix.c_str( ) );
		}
	} else if( rxPacket.responseType == RESP_LED_GREEN ) {
		if( rxPacket.pl.u8[0] == 1 ) {
			printf( "%s INFO    : green led is on\n", prefix.c_str( ) );
		} else {
			printf( "%s INFO    : green led is off\n", prefix.c_str( ) );
		}
	} else if ( rxPacket.responseType == RESP_DEFAULT ) {
		if( rxPacket.payloadSize != 40 ) {
			printf( "%s ERROR   : default response packet size is %d but should be %d\n", prefix.c_str( ), rxPacket.payloadSize, 40 );
		} else {
			if( defaultRespCounter == 1000 ) { // prevent cluttering up screen
				defaultRespCounter = 0;
				float currentA = 0;
				if( currentA < 0 ) { currentA = 0; }
				float currentB = 0;
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
					printf( "%s INFO    : disp %8d set %5d, vel %5d, pidErr %4d, intgrl %7d, pid %10d, pwm %6d, timeM %6d, timeC %6d, curA %4.2f, curB %4.2f, volt %3.2f, tempB %4.2f, tempM %4.2f\n",
						prefix.c_str( ),
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
					printf( "%s INFO    :               set %5d,            pidErr %4d, intgrl %7d, pid %10d, pwm %6d, timeM %6d, timeC %6d, curA %4.2f, curB %4.2f, volt %3.2f, tempB %4.2f, tacho %4.2f %5d, angle %4.2f %5d\n",
						prefix.c_str( ),
						primarySetPoint,
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
				printf( "%s ERROR   : loopback value is 0x%02x but should be 0x%02x\n", prefix.c_str( ), rxPacket.pl.u8[ii], ii );;
			}
		}
	} else if ( rxPacket.responseType == RESP_MOTOR_TIMEOUT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "%s ERROR   : invalid payload size for motor timeout response %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			printf( "%s INFO    : motor will be disabled if no valid packet has been received within %.1f ms\n", prefix.c_str( ), 2.5 * rxPacket.pl.u16[0] );
		}
	} else if ( rxPacket.responseType == RESP_PWM_LIMIT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "%s ERROR   : invalid payload size for pwm limit response %d\n", prefix.c_str( ), rxPacket.payloadSize );;
		} else {
			printf( "%s INFO    : pwm will be limited to %d\n", prefix.c_str( ), rxPacket.pl.u16[0] );
		}
	} else if ( rxPacket.responseType == RESP_PWM_DELTA ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "%s ERROR   : invalid payload size for pwm delta response %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			printf( "%s INFO    : pwm delta (maximal increase per 2.5ms) is limited to %d\n", prefix.c_str( ), rxPacket.pl.u16[0] );
		}
	} else if ( rxPacket.responseType == RESP_PID_ANGLE_PROPERTIES ) {
		if( rxPacket.payloadSize != 8 ) {
			printf( "%s ERROR   : invalid payload size for the pid angle properties %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			printf( "%s INFO    : pid angle value are set to: Kp %d, Ki %d, Kd %d iTh %d\n", prefix.c_str( ), rxPacket.pl.u16[0], rxPacket.pl.u16[1], rxPacket.pl.u16[2], rxPacket.pl.u16[3] );
		}
	} else if ( rxPacket.responseType == RESP_PID_PRIMARY_PROPERTIES ) {
		if( rxPacket.payloadSize != 8 ) {
			printf( "%s ERROR   : invalid payload size for the pid primary properties %d\n", prefix.c_str( ), rxPacket.payloadSize );;
		} else {
			printf( "%s INFO    : pid primary value are set to: Kp %d, Ki %d, Kd %d iTh %d\n", prefix.c_str( ), rxPacket.pl.u16[0], rxPacket.pl.u16[1], rxPacket.pl.u16[2], rxPacket.pl.u16[3] );
		}
	} else if ( rxPacket.responseType == RESP_PRIMARY_SETPOINT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "%s ERROR   : invalid payload size for the primary setpoint response %d\n", prefix.c_str( ), rxPacket.payloadSize );;
		} else {
			printf( "%s INFO    : primary setpoint value %d\n", prefix.c_str( ), rxPacket.pl.s16[0] );
		}
	} else if ( rxPacket.responseType == RESP_ANGLE_SETPOINT ) {
		if( rxPacket.payloadSize != 2 ) {
			printf( "%s ERROR   : invalid payload size for the angle setpoint response %d\n", prefix.c_str( ), rxPacket.payloadSize );;
		} else {
			printf( "%s INFO    : angle setpoint value %d\n", prefix.c_str( ), rxPacket.pl.s16[0] );
		}
	} else if ( rxPacket.responseType == RESP_ANGLE_DIRECTION ) {
		if( rxPacket.payloadSize != 1 ) {
			printf( "%s ERROR   : invalid payload size for the angle direction %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			if( rxPacket.pl.u8[0] == 0 ) {
				printf( "%s INFO    : angle direction behavior NOT inverted\n", prefix.c_str( ) );
			} else {
				printf( "%s INFO    : angle direction behavior inverted\n", prefix.c_str( ) );
			}
		}
	} else if ( rxPacket.responseType == RESP_MODE ) {
		if( rxPacket.payloadSize != 1 ) {
			printf( "%s ERROR   : invalid payload size for the mode %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			if( rxPacket.pl.u8[0] == MODE_PID_ANGLE ) { printf( "%s INFO    : mode angle = first pid input is angle + setpoint, second pid input is first pid + tacho\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_PID_ENCODER ) { printf( "%s INFO    : mode encoder = pid input is encoder + setpoint\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_PID_TACHO ) { printf( "%s INFO    : mode tacho = pid input is tacho + setpoint\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_PWM_ONLY ) { printf( "%s INFO    : mode pwm = no pid, pwm is directly controlled from setpoint\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_COMMUNICATION_TIMEOUT ) { printf( "%s INFO    : mode timeout = motor disabled because communication timeout\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_DISABLE_MOTOR_POWER ) { printf( "%s INFO    : mode disabled = motor disabled (after reset / or Linux software initialization)\n", prefix.c_str( ) ); }
			else if( rxPacket.pl.u8[0] == MODE_ZERO_CURRENT_CALIBRATION ) { printf( "%s INFO    : mode calibration = motor disabled for zero current calibration of drv8301\n", prefix.c_str( ) ); }
			else { printf( "%s ERROR   : no mode %d set, abort\n", prefix.c_str( ), rxPacket.pl.u8[0] ); exit(1); }
		}
	} else if ( rxPacket.responseType == RESP_ANGLE_TACHO_ZERO ) {
		if( rxPacket.payloadSize != 4 ) {
			printf( "%s ERROR   : invalid payload size for angle tacho zero value %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			printf( "%s INFO    : angle zero %d, tacho zero %d\n", prefix.c_str( ), rxPacket.pl.u16[0], rxPacket.pl.u16[1] );
		}
	} else if ( rxPacket.responseType == RESP_DRV8301 ) {
		if( rxPacket.payloadSize != 12 ) {
			printf( "%s ERROR   : invalid payload size for the drv8301 %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			uint16_t status1 = rxPacket.pl.u16[0];
			uint16_t status2 = rxPacket.pl.u16[1];
			uint16_t control1 = rxPacket.pl.u16[2];
			uint16_t control1Strip  = 0x07ff & control1; // control value read from drv8301, but without the address
			uint16_t control2 = rxPacket.pl.u16[3];
			uint16_t control2Strip  = 0x07ff & control2; // control value read from drv8301, but without the address
			uint16_t setControl1 = 0x07fb & rxPacket.pl.u16[4]; // the b in 0x07fb because that value get's reset to zero after write
			uint16_t setControl2 = 0x07ff & rxPacket.pl.u16[5];
			if( control1Strip != setControl1 ) { printf( "%s ERROR   : drv8301 control1 0x%04x differs from set control1 0x%04x\n", prefix.c_str( ), control1Strip, setControl1 ); }
			if( control2Strip != setControl2 ) { printf( "%s ERROR   : drv8301 control2 0x%04x differs from set control2 0x%04x\n", prefix.c_str( ), control2Strip, setControl2 ); }

			printf( "%s INFO    : drv8301 status  1 (0x%04x):", prefix.c_str( ), status1 );
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
			printf( "%s INFO    : drv8301 status  2 (0x%04x):", prefix.c_str( ), status2 );
			if( (status2 & 0xf) == 1 ) { printf( " device ID %d", status2 & 0xf ); } else { printf( " ERROR   : wrong device ID %d", status2 & 0xf ); }
			if( ((status2>>7) & 1) == 1 ) { printf( ", overvoltage protection limit (GVDD_OV)" ); }
			if( ((status2>>11) & 3) != 1 ) { printf( ", WRONG INDEX SHALL NEVEN HAPPEN!!" ); }
			printf( "\n" );
			printf( "%s INFO    : drv8301 control 1 (0x%04x):", prefix.c_str( ), control1 );
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

			if( ((control1>>4) & 3) == 0 ) { printf( ", over current protection mode current limit" ); }
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

			printf( "%s INFO    : drv8301 control 2 (0x%04x):", prefix.c_str( ), control2 );

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
			printf( "%s ERROR   : invalid payload size for board response %d\n", prefix.c_str( ), rxPacket.payloadSize );
		} else {
			printf( "%s INFO    : Device ID 0x%04x, Vendor ID 0x%04x, Application ID 0x%04x\n", prefix.c_str( ), rxPacket.pl.u16[0], rxPacket.pl.u16[1], rxPacket.pl.u16[2] );
			printf( "%s INFO    : Hardware Version 0x%04x (%d), Software Version 0x%04x (%d)\n", prefix.c_str( ), rxPacket.pl.u16[3], rxPacket.pl.u16[3], rxPacket.pl.u16[4], rxPacket.pl.u16[4] );
			// placeholder rxPacket.pl.u16[5]
			printf( "%s INFO    : Xmega Device ID 0x%08x, Xmega Serial 0x%08x, CPU Clock %d Hz\n", prefix.c_str( ), rxPacket.pl.u32[3],rxPacket.pl.u32[4], rxPacket.pl.u32[5] );

			if( rxPacket.pl.u16[3] != HARDWARE_VERSION ) {
				printf( "%s ERROR   : invalid different hardware version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", prefix.c_str( ), HARDWARE_VERSION, HARDWARE_VERSION, rxPacket.pl.u16[3], rxPacket.pl.u16[3] ); exit(1);
			}
			if( rxPacket.pl.u16[4] != SOFTWARE_VERSION ) {
				printf( "%s ERROR   : invalid different software version between linux 0x%04x (%d) and board 0x%04x (%d), abort\n", prefix.c_str( ), SOFTWARE_VERSION, SOFTWARE_VERSION, rxPacket.pl.u16[4], rxPacket.pl.u16[4] ); exit(1);
			}
			if( applicationId == 255 ) {
				// applicationId was not set, then it should be a value from 1 to 6
				if( rxPacket.pl.u16[2] == 0 ) {
					printf( "%s ERROR   : invalid Application ID %d, probably short, abort\n", prefix.c_str( ), rxPacket.pl.u16[2] ); exit(1);
				}
				if( rxPacket.pl.u16[2] == 7 ) {
					printf( "%s ERROR   : invalid Application ID %d, probably open, abort\n", prefix.c_str( ), rxPacket.pl.u16[2] ); exit(1);
				}
			} else {
				if( rxPacket.pl.u16[2] != applicationId ) {
					printf( "%s ERROR   : invalid Application ID %d, expected %d, abort\n", prefix.c_str( ), rxPacket.pl.u16[2], applicationId ); exit(1);
				}
			}
		}
	} else {
		printf( "%s INFO    : receive %3d | ", prefix.c_str( ), packetSize );
		char *rxPacketPointer = (char *) &rxPacket; // use char pointer to index the individual bytes of the rxPacket
		for( int ii = 0; ii < packetSize; ii++ ) {
			printf( "%02x ", (uint8_t) *rxPacketPointer );
 			if( ii == (FROM_BOARD_PACKET_HEADER_SIZE-1) ) { printf( "| " ); }
			rxPacketPointer++;
		}
 		printf( "\n" );
	}
}

// function to convert the raw adc value from the xmega to ADC voltage
float motorBoard::adcToVoltage( float adc ) {
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
float motorBoard::voltageToPowerSuppy( float adcVoltage ) {
	return adcVoltage * ( 3.0 + 30.0 ) / 3.0; // values from schematic
}

// function to convert the adc voltage to ntc resistance
float motorBoard::voltageToNtcResistance( float adcVoltage ) {
	// for as well board temperature NTC as motor NTC temperature
	// the pull up resistor is 4k7 and connected to 3v3
	// the adcVoltage is measured over the ntc resistor
	float vcc3v3 = 3.27;
	float pullUp = 4700;
	// return current resistance value of ntc resistor
	return adcVoltage * pullUp / ( vcc3v3 - adcVoltage );
}

// function to convert the board ntc resistance to board temperature
float motorBoard::resistanceToBoardTemperature( float ntcResistance ) {
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
float motorBoard::resistanceToMotorTemparature( float ntcResistance ) {
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


void motorBoard::printRemoteErrors( string prefix, fromBoardPacketT rxPacket ) {
	uint16_t commError = rxPacket.pl.u16[0];
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_WRITE) != 0 ) { printf( "%s ERROR   : far end communication from board write!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : far end communication from board out of range!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_END_CHECK) != 0 ) { printf( "%s ERROR   : far end communication from board end check!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN) != 0 ) { printf( "%s ERROR   : far end communication from board sof overwritten!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW) != 0 ) { printf( "%s ERROR   : far end communication to board size overflow!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_OVERFLOW) != 0 ) { printf( "%s ERROR   : far end communication to board overflow!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_CHECKSUM) != 0 ) { printf( "%s ERROR   : far end communication to board checksum!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_END_CHECK) != 0 ) { printf( "%s ERROR   : far end communication to board end check!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT) != 0 ) { printf( "%s ERROR   : far end communication to board sof alignment!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND) != 0 ) { printf( "%s ERROR   : far end communication to board illegal command!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC) != 0 ) { printf( "%s ERROR   : far end communication to board ack id out of sync!\n", prefix.c_str( ) ); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN) != 0 ) { printf( "%s ERROR   : far end communication to board packet overwritten!\n", prefix.c_str( ) ); }
	// a far end receive buffer overflow shall never happen, exit to it directly visible
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW) != 0 ) { printf( "%s ERROR   : far end communication to board buffer overflow!\n", prefix.c_str( ) ); exit(EXIT_FAILURE); }
	if( ( commError & COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE) != 0 ) { printf( "%s ERROR   : far end communication to board incorrect amount of bytes received!\n", prefix.c_str( ) ); }

	uint16_t encErrror = rxPacket.pl.u16[1];
	if( ( encErrror & ENCODER_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s RROR   : encoder not initialized!\n", prefix.c_str( ) ); }
	if( ( encErrror & ENCODER_ERROR_VELOCITY_MAX_NEGATIVE) != 0 ) { printf( "%s ERROR   : encoder maximal negative motor speed!\n", prefix.c_str( ) ); }
	if( ( encErrror & ENCODER_ERROR_VELOCITY_MAX_POSITIVE) != 0 ) { printf( "%s ERROR   : encoder maximal positive motor speed!\n", prefix.c_str( ) ); }

	uint16_t pidError = rxPacket.pl.u16[2];
	if( ( pidError & PID_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s ERROR   : pid not initialized!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : primary pid error value out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : primary pid integral out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : primary pid derivative out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : primary pid result out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : angle pid error value out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : angle pid integral out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : angle pid derivative out of range!\n", prefix.c_str( ) ); }
	if( ( pidError & PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE) != 0 ) { printf( "%s ERROR   : angle pid result out of range!\n", prefix.c_str( )); }

	uint16_t pwmError = rxPacket.pl.u16[3];
	if( ( pwmError & PWM_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s ERROR   : pwm not initialized!\n", prefix.c_str( ) ); }
	if( ( pwmError & PWM_ERROR_POSITIVE_LIMIT) != 0 ) { printf( "%s ERROR   : pwm error positive limit!\n", prefix.c_str( ) ); }
	if( ( pwmError & PWM_ERROR_NEGATIVE_LIMIT) != 0 ) { printf( "%s ERROR   : pwm error negative limit!\n", prefix.c_str( ) ); }
	if( ( pwmError & PWM_ERROR_POSITIVE_DELTA) != 0 ) { printf( "%s ERROR   : pwm error positive delta (increase to fast)!\n", prefix.c_str( ) ); }
	if( ( pwmError & PWM_ERROR_NEGATIVE_DELTA) != 0 ) { printf( "%s ERROR   : pwm error negative delta (reverse to fast)!\n", prefix.c_str( ) ); }

	uint16_t safetyError = rxPacket.pl.u16[4];
	if( ( safetyError & SAFETY_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s ERROR   : safety not initialized!\n", prefix.c_str( ) ); }
	if( ( safetyError & SAFETY_COMMUNICATION_TIMEOUT) != 0 ) { printf( "%s ERROR   : safety says board did not receive new packet in time! (motorTimeout)\n", prefix.c_str( ) ); }
	if( ( safetyError & SAFETY_ANGLE_TACHO_UNINITIALZED) != 0 ) { printf( "%s ERROR   : safety says un-initialized angle and or tacho used!\n", prefix.c_str( ) ); }
	// the next error might appear when noise on the hall sensor
	if( ( safetyError & SAFETY_ANGLE_VALUE_TOO_HIGH) != 0 ) { printf( "%s ERROR   : safety says angle value too high!, probaly unconnected hall sensor\n", prefix.c_str( ) ); }
	if( ( safetyError & SAFETY_ANGLE_VALUE_TOO_LOW) != 0 ) { printf( "%s ERROR   : safety says angle value too low!, probably short on hall sensor\n", prefix.c_str( ) ); }
	if( ( safetyError & SAFETY_TACHO_VALUE_TOO_HIGH) != 0 ) { printf( "%s ERROR   : safety says tacho value too high!, abort\n", prefix.c_str( ) ); exit(1); }
	if( ( safetyError & SAFETY_TACHO_VALUE_TOO_LOW) != 0 ) { printf( "%s ERROR   : safety says tacho value too low!, abort\n", prefix.c_str( ) ); exit(1); }

	uint16_t schedulerError = rxPacket.pl.u16[5];
	if( ( schedulerError & SCHEDULER_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s ERROR   : scheduler not initialized!\n", prefix.c_str( ) ); }
	if( ( schedulerError & SCHEDULER_ERROR_MEASURE_TIME_EXPIRED) != 0 ) { printf( "%s ERROR   : scheduler measurement cycle to long!\n", prefix.c_str( ) ); }
	if( ( schedulerError & SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED) != 0 ) { printf( "%s ERROR   : scheduler calculation cycle to long!\n", prefix.c_str( ) ); }
	if( ( schedulerError & SCHEDULER_ERROR_TEST) != 0 ) { printf( "%s ERROR   : scheduler test\n", prefix.c_str( ) ); }

	uint16_t drv8301Error = rxPacket.pl.u16[6];
	if( ( drv8301Error & DRV8301_ERROR_INIT_NOT_PERFORMED) != 0 ) { printf( "%s ERROR   : drv8301 not initialized!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETLC_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET low c over current (FETLC_OC) !\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETHC_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET high c over current (FETHC_OC)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETLB_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET low b over current (FETLB_OC)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETHB_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET high b over current (FETHB_OC)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETLA_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET low a over current (FETLA_OC)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FETHA_OC) != 0 ) { printf( "%s ERROR   : drv8301 error FET high a over current (FETHA_OC)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_OTW) != 0 ) { printf( "%s ERROR   : drv8301 error junction overtemperature warning (OTW)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_OTSD) != 0 ) { printf( "%s ERROR   : drv8301 error junction overtemperature shutdown (OTSD)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_PVDD_UV) != 0 ) { printf( "%s ERROR   : drv8301 error power supply undervoltage protection (PVDD_UV)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_GVDD_UV) != 0 ) { printf( "%s ERROR   : drv8301 error internal gate driver undervoltage protection (GVDD_UV)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_FAULT) != 0 ) { printf( "%s ERROR   : drv8301 error shutdown occurred (FAULT)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_DEVICE_ID) != 0 ) { printf( "%s ERROR   : drv8301 error wrong Device ID!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_GVDD_OV) != 0 ) { printf( "%s ERROR   : drv8301 error overvoltage protection limit (GVDD_OV)!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_CONTROL_REGISTER1) != 0 ) { printf( "%s ERROR   : drv8301 write to control register 1 unsuccessful!\n", prefix.c_str( ) ); }
	if( ( drv8301Error & DRV8301_ERROR_CONTROL_REGISTER2) != 0 ) { printf( "%s ERROR   : drv8301 write to control register 2 unsuccessful!\n", prefix.c_str( ) ); }
}
