 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

