// Copyright 2018-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAM_SYS_RECEIVE_HPP
#define CAM_SYS_RECEIVE_HPP

#include <inttypes.h>
#include <vector>

#include "visionObjects.hpp"


class mlAdapter {
public:
	mlAdapter();

private:
	void receive(); // blocking call, run in separate thread
	void decode();
	void print();

	struct camPacketT {
	    uint8_t id;
	    uint8_t cnt;
	    uint16_t size; // packet size including header in bytes, payload size maximal is 65503 instead of 65536, see below
	    union {
	        // 2^16 = 64 KiB = 65536 bytes, but limited 65503 bytes, see below
	        uint8_t u8[1 << 16]; // unsigned payload
	        uint16_t u16[1 << 15];
	        uint32_t u32[1 << 14];
	        uint64_t u64[1 << 13];
	        int8_t s8[1 << 16]; // signed payload
	        int16_t s16[1 << 15];
	        int32_t s32[1 << 14];
	        int64_t s64[1 << 13];
	        float f32[1 << 14];
	        double d64[1 << 13];
	    } pl;
	}__attribute__((packed));

	int udpSocket;
	camPacketT rxPacket;
	uint8_t rxPacketCntExpected;
	bool rxPacketCntEnabled;

    // sharedType / RtDB data
	RtDB2 *_rtdb;
    visionObjects mlObjects;
};

#endif
