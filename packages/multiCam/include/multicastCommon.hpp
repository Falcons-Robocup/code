// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2017-2019 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MULTICAST_COMMON
#define MULTICAST_COMMON

#include <cstdint>

typedef struct {
    uint8_t type;
    uint8_t cnt;
    uint16_t size; // packet size including header in bytes
    union {
        uint8_t u8[1 << 16]; // unsigned payload
        uint16_t u16[1 << 15];
        uint32_t u32[1 << 14];
        uint64_t u64[1 << 13];
        int8_t s8[1 << 16]; // signed payload
        int16_t s16[1 << 15];
        int32_t s32[1 << 14];
        int64_t s64[1 << 13];
        float f32[1 << 13];
        double d64[1 << 12];
    } pl;
}__attribute__((packed)) packetT;

#define HEADER_SIZE 4

#define TYPE_STATS 4
#define TYPE_LINEPOINTS 5
#define TYPE_LOCALIZATION 6
#define TYPE_BALLDETECTION 7
#define TYPE_BALLFARDETECTION 8
#define TYPE_OBSTACLES 9
#define TYPE_GOOD_ENOUGH_LOC 10
#define TYPE_CYANDETECTION 11
#define TYPE_MAGENTADETECTION 12

#endif
