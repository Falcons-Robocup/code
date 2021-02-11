// Copyright 2018-2019 Andre Pool (Falcons)
// SPDX-License-Identifier: Apache-2.0
// Copyright 2018 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef CAM_PACKET_HPP
#define CAM_PACKET_HPP

#include <inttypes.h>

#define IPV4_MAX_SIZE 65535 // 2^16 - 1, TODO: why not 65536 ?
#define IPV4_IP_HEADER 20
#define UDP_PACKET_MAX_SIZE (IPV4_MAX_SIZE - IPV4_IP_HEADER) // 65515
#define UDP_HEADER_SIZE 8
#define UDP_PACKET_PAYLOAD_SIZE (UDP_PACKET_MAX_SIZE - UDP_HEADER_SIZE) // 65507

typedef struct {
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
}__attribute__((packed)) camPacketT;

#define CAM_PACKET_HEADER_SIZE 4
#define CAM_PACKET_PAYLOAD_SIZE (UDP_PACKET_PAYLOAD_SIZE - CAM_PACKET_HEADER_SIZE) // 65503

#endif // CAM_PACKET_HPP

