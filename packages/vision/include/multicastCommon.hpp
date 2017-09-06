 /*** 
 2014 - 2017 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 // Copyright 2017 Andre Pool
// Licensed under the Apache License version 2.0
// You may not use this file except in compliance with this License
// You may obtain a copy of the License at http://www.apache.org/licenses/LICENSE-2.0

#ifndef MULTICAST_COMMON
#define MULTICAST_COMMON

#include <cstdint>

typedef struct {
	uint8_t type;
	uint8_t cnt;
	uint16_t size; // packet size including header in bytes
	union {
		uint8_t u8[1<<16]; // unsigned payload
		uint16_t u16[1<<15];
		uint32_t u32[1<<14];
		uint64_t u64[1<<13];
		int8_t s8[1<<16]; // signed payload
		int16_t s16[1<<15];
		int32_t s32[1<<14];
		int64_t s64[1<<13];
		float f32[1<<13];
		double d64[1<<12];
	} pl;
} __attribute__((packed)) packetT;

#define HEADER_SIZE 4

#define TYPE_STATS 4
#define TYPE_LINEPOINTS 5
#define TYPE_LOCALIZATION 6
#define TYPE_BALLDETECTION 7
#define TYPE_OBSTACLES 8
#define TYPE_GOOD_ENOUGH_LOC 9
#define TYPE_CYANDETECTION 10
#define TYPE_MAGENTADETECTION 11

#endif
