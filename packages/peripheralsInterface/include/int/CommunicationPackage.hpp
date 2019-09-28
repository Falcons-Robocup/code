 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * CommunicationPackage.hpp
 *
 *  Created on: Jun 7, 2016
 *      Author: Edwin Schreuder
 */

#ifndef INCLUDE_INT_COMMUNICATIONPACKAGE_HPP_
#define INCLUDE_INT_COMMUNICATIONPACKAGE_HPP_

#include <string>

// Warning: never use hard coded command numbers but instead use this list!
// The list is subject to change and commands are alphabetic sorted.
// When the list is changed the software version also should change.
// The software version can be queried through the board command.
// For that reason the get board command needs stay at position 1
enum CommandList {
	CMD_AA_DO_NOT_USE_ZERO = 0,
	CMD_AA_GET_BOARD, // should stay at position 1
	CMD_CLEAR_ERRORS,
	CMD_GET_ERRORS,
	CMD_GET_GAIN, // TODO: remove, not used anymore
	CMD_GET_LED_GREEN,
	CMD_GET_LED_YELLOW,
	CMD_GET_MOTOR_TIMEOUT,
	CMD_GET_PID_PRIMARY_PROPERTIES,
	CMD_GET_PWM_LIMIT,
	CMD_GET_PRIMARY_SETPOINT,
	CMD_LOOPBACK,
	CMD_SET_GAIN, // TODO: remove, not used anymore
	CMD_SET_LED_GREEN,
	CMD_SET_LED_YELLOW,
	CMD_SET_MOTOR_TIMEOUT,
	CMD_SET_PID_PRIMARY_PROPERTIES,
	CMD_SET_PWM_MANUAL,// TODO: remove, not used anymore
	CMD_SET_PWM_LIMIT,
	CMD_SET_PRIMARY_SETPOINT,

	CMD_GET_PWM_MANUAL, // TODO: remove, not used anymore
	CMD_GET_DRV8301,
	CMD_SET_DRV8301,
	CMD_GET_PWM_DELTA,
	CMD_SET_PWM_DELTA,
	CMD_GET_MODE, // e.g. wheel motors on encoder, ball handler motor on tacho or set a fixed pwm value
	CMD_SET_MODE,
	CMD_GET_ANGLE_TACHO_ZERO,
	CMD_GET_PID_ANGLE_PROPERTIES,
	CMD_SET_PID_ANGLE_PROPERTIES,
	CMD_GET_ANGLE_SETPOINT,
	CMD_SET_ANGLE_SETPOINT,
	CMD_GET_ANGLE_DIRECTION,
	CMD_SET_ANGLE_DIRECTION,

	CMD_ZZ_DO_NOT_USE_LAST
};

enum ResponseList {
	RESP_AA_DO_NOT_USE_ZERO = 0,
	RESP_AA_BOARD, // should stay at position 1
	RESP_DEFAULT,
	RESP_ERROR,
	RESP_GAIN, // TODO: remove, not used anymore
	RESP_LED_GREEN,
	RESP_LED_YELLOW,
	RESP_LOOPBACK,
	RESP_MOTOR_TIMEOUT,
	RESP_PID_PRIMARY_PROPERTIES,
	RESP_PWM_LIMIT,
	RESP_PRIMARY_SETPOINT,
	RESP_PWM_MANUAL, // TODO: remove, not used anymore
	RESP_DRV8301,
	RESP_PWM_DELTA,
	RESP_MODE,
	RESP_ANGLE_TACHO_ZERO,
	RESP_PID_ANGLE_PROPERTIES,
	RESP_ANGLE_SETPOINT,
	RESP_ANGLE_DIRECTION,
	RESP_ZZ_DO_NOT_USE_LAST
};

enum ModeList {
	MODE_AA_DO_NOT_USE_ZERO = 0,
	MODE_PID_ANGLE,
	MODE_PID_ENCODER,
	MODE_PID_TACHO,
	MODE_PWM_ONLY,
	MODE_COMMUNICATION_TIMEOUT,
	MODE_DISABLE_MOTOR_POWER,
	MODE_ZERO_CURRENT_CALIBRATION,
	MODE_ZZ_DO_NOT_USE_LAST
};

// errors for the communication functions
// each bit represents a specific communication error
#define COMMUNICATION_ERROR_FROM_BOARD_WRITE               0x0001
#define COMMUNICATION_ERROR_FROM_BOARD_OUT_OF_RANGE        0x0002
#define COMMUNICATION_ERROR_FROM_BOARD_END_CHECK           0x0004
#define COMMUNICATION_ERROR_FROM_BOARD_SOF_OVERWRITTEN     0x0008
#define COMMUNICATION_ERROR_TO_BOARD_SIZE_OVERFLOW         0x0010
#define COMMUNICATION_ERROR_TO_BOARD_OVERFLOW              0x0020
#define COMMUNICATION_ERROR_TO_BOARD_CHECKSUM              0x0040
#define COMMUNICATION_ERROR_TO_BOARD_END_CHECK             0x0080
#define COMMUNICATION_ERROR_TO_BOARD_SOF_ALIGNMENT         0x0100
#define COMMUNICATION_ERROR_TO_BOARD_ILLEGAL_COMMAND       0x0200
#define COMMUNICATION_ERROR_TO_BOARD_ACK_ID_OUT_OF_SYNC    0x0400
#define COMMUNICATION_ERROR_TO_BOARD_PACKET_OVERWRITTEN    0x0800
#define COMMUNICATION_ERROR_TO_BOARD_BUFFER_OVERFLOW       0x1000
#define COMMUNICATION_ERROR_TO_BOARD_PAYLOADSIZE           0x2000

#define ENCODER_ERROR_INIT_NOT_PERFORMED                   0x0001
#define ENCODER_ERROR_VELOCITY_MAX_NEGATIVE                0x0002
#define ENCODER_ERROR_VELOCITY_MAX_POSITIVE                0x0004

#define PID_ERROR_INIT_NOT_PERFORMED           	           0x0001
#define PID_ERROR_PRIMARY_CURRENT_ERROR_OUT_OF_RANGE       0x0002
#define PID_ERROR_PRIMARY_INTEGRAL_OUT_OF_RANGE            0x0004 // not used anymore
#define PID_ERROR_PRIMARY_DERIVATIVE_OUT_OF_RANGE          0x0008
#define PID_ERROR_PRIMARY_RESULT_OUT_OF_RANGE              0x0010
#define PID_ERROR_ANGLE_CURRENT_ERROR_OUT_OF_RANGE         0x0020
#define PID_ERROR_ANGLE_INTEGRAL_OUT_OF_RANGE              0x0040 // not used anymore
#define PID_ERROR_ANGLE_DERIVATIVE_OUT_OF_RANGE            0x0080
#define PID_ERROR_ANGLE_RESULT_OUT_OF_RANGE                0x0100
#define PID_ERROR_REQUEST_CLEAR_STATE                      0x8000

#define SAFETY_ERROR_INIT_NOT_PERFORMED                    0x0001
#define SAFETY_COMMUNICATION_TIMEOUT                       0x0002
#define SAFETY_ANGLE_TACHO_UNINITIALZED                    0x0004
#define SAFETY_ANGLE_VALUE_TOO_HIGH                        0x0008
#define SAFETY_ANGLE_VALUE_TOO_LOW                         0x0010
#define SAFETY_TACHO_VALUE_TOO_HIGH                        0x0020
#define SAFETY_TACHO_VALUE_TOO_LOW                         0x0040

#define SCHEDULER_ERROR_INIT_NOT_PERFORMED                 0x0001
#define SCHEDULER_ERROR_MEASURE_TIME_EXPIRED               0x0002
#define SCHEDULER_ERROR_CALCULATE_TIME_EXPIRED             0x0004
#define SCHEDULER_ERROR_TEST                               0x8000

#define PWM_ERROR_INIT_NOT_PERFORMED                       0x0001
#define PWM_ERROR_POSITIVE_LIMIT                           0x0002
#define PWM_ERROR_NEGATIVE_LIMIT                           0x0004
#define PWM_ERROR_POSITIVE_DELTA                           0x0008 // not used anymore
#define PWM_ERROR_NEGATIVE_DELTA                           0x0010 // not used anymore
#define PWM_ERROR_REQUEST_CLEAR_STATE                      0x8000

#define DRV8301_ERROR_INIT_NOT_PERFORMED                   0x0001
#define DRV8301_ERROR_FETLC_OC                             0x0002
#define DRV8301_ERROR_FETHC_OC                             0x0004
#define DRV8301_ERROR_FETLB_OC                             0x0008
#define DRV8301_ERROR_FETHB_OC                             0x0010
#define DRV8301_ERROR_FETLA_OC                             0x0020
#define DRV8301_ERROR_FETHA_OC                             0x0040
#define DRV8301_ERROR_OTW                                  0x0080
#define DRV8301_ERROR_OTSD                                 0x0100
#define DRV8301_ERROR_PVDD_UV                              0x0200
#define DRV8301_ERROR_GVDD_UV                              0x0400
#define DRV8301_ERROR_FAULT                                0x0800
#define DRV8301_ERROR_DEVICE_ID                            0x1000
#define DRV8301_ERROR_GVDD_OV                              0x2000
#define DRV8301_ERROR_CONTROL_REGISTER1                    0x4000
#define DRV8301_ERROR_CONTROL_REGISTER2                    0x8000

#define SOF												0x5A
#define FROM_BOARD_PACKET_HEADER_SIZE 					7

class CommunicationPackage
{
public:
	CommunicationPackage(size_t payloadIndex);
	virtual ~CommunicationPackage();

	virtual unsigned char * getBuffer();
	size_t getLength();

	unsigned char getAckId();
	unsigned char getPayloadSize();

	void print();

	template <typename T> T getData(size_t position);

protected:
	size_t payloadLength;
	unsigned char buffer[264];

	size_t payloadIndex;

	unsigned char getCheckSum();
	unsigned char calculateCheckSum();
};

class ReceivePackage : public CommunicationPackage
{
public:
	ReceivePackage(unsigned char * package, size_t size);

	unsigned char getBufferSpace();
	unsigned char getFeedbackId();
	ResponseList getResponseType();

	bool isPacketValid();
};

class TransmitPackage : public CommunicationPackage
{
public:
	TransmitPackage(CommandList command);

	unsigned char * getBuffer();
	void setAckId(unsigned char ackId);
	CommandList getCommand();
	template <typename T> void addData(T data);

private:
	void setPayloadSize(unsigned char payloadSize);
	void setCheckSum(unsigned char checksum);
};

#endif /* INCLUDE_INT_COMMUNICATIONPACKAGE_HPP_ */
