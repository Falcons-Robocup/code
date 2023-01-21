// Copyright 2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef EXPORTER_HPP
#define EXPORTER_HPP

#include <QUdpSocket>
#include <QWidget>

class mainWidget; //forward declaration

class exporter : public QWidget {
public:
   explicit exporter(mainWidget *parent);
   ~exporter();
   void update(const size_t camera);

   float xScale, yScale;

private:
   struct  camPacketT {
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

   void routeCheck();
   void sendFrame();

   mainWidget *mainW = nullptr;

   QUdpSocket *udpSocket = nullptr;
   camPacketT txPacket;

   QHostAddress groupAddress;
   quint16 port;
};

#endif // EXPORTER_HPP


