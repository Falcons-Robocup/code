// Copyright 2015, 2016 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef INCLUDED_COMMUNICATION_H
#define INCLUDED_COMMUNICATION_H

#include <stdbool.h>
#include "global.h"


#define SOF 0x5a

// Warning: never use hard coded command numbers but instead use this list!
// The list is subject to change and commands are alphabetic sorted.
// When the list is changed the software version also should change.
// The software version can be queried through the board command.
// For that reason the get board command needs stay at position 1
typedef enum commandList {
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
	CMD_SET_PWM_MANUAL, // TODO: remove, not used anymore
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
} commandListT;

typedef enum responseList {
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
} responseListT;

typedef enum modeList {
	MODE_AA_DO_NOT_USE_ZERO = 0,
	MODE_PID_ANGLE,
	MODE_PID_ENCODER,
	MODE_PID_TACHO,
	MODE_PWM_ONLY,
	MODE_COMMUNICATION_TIMEOUT,
	MODE_DISABLE_MOTOR_POWER,
	MODE_ZERO_CURRENT_CALIBRATION,
	MODE_ZZ_DO_NOT_USE_LAST
} modeListT;


typedef struct
{
	uint8_t sof;
	uint8_t ackId;
	uint8_t command;
	uint8_t checksum; // sum of all bytes in the packet excluding the checksum byte itself
	uint8_t payloadSize; // in bytes
	union // maximal payload size is 255 bytes
	{
		uint8_t u8[255]; // unsigned payload
		uint16_t u16[127];
		uint32_t u32[63];
		uint64_t u64[31];
		int8_t s8[255]; // signed payload
		int16_t s16[127];
		int32_t s32[63];
		int64_t s64[31];
	} pl;
	uint8_t endCheck; // check out of packet write
} __attribute__((packed)) toBoardPacketT;

#define TO_BOARD_PACKET_HEADER_SIZE 5
#define TO_BOARD_PACKET_SIZE (TO_BOARD_PACKET_HEADER_SIZE + 255)

typedef struct
{
	uint8_t sof; // keeps default value
	uint8_t ackId; // increment with wrap amount
	uint8_t feedbackId; // used by PC to accurate calculate board RX buffer size
	uint8_t bufferSpace; // bytes that can be received
	uint8_t responseType;
	uint8_t checksum; // sum of all bytes in the packet excluding the checksum byte itself
	uint8_t payloadSize; // in bytes
	union // maximal payload size is 255 bytes
	{
		uint8_t u8[255]; // unsigned payload
		uint16_t u16[127];
		uint32_t u32[63];
		uint64_t u64[31];
		int8_t s8[255]; // signed payload
		int16_t s16[127];
		int32_t s32[63];
		int64_t s64[31];
	} pl;
	uint8_t endCheck; // check out of packet write
} __attribute__((packed)) fromBoardPacketT;

#define FROM_BOARD_PACKET_HEADER_SIZE 7
#define FROM_BOARD_PACKET_SIZE (FROM_BOARD_PACKET_HEADER_SIZE + 255)

void initCommunication();

void taskCommunication();

void sendPacket();

void receivePacket();

void rxPacketAckIdCheck();

bool rxPacketChecksumFail();

uint16_t getCommunicationError();

void clearCommunicationError(uint16_t value);

void setRxBufferOverflowError();

#endif /* INCLUDED_COMMUNICATION_H */
