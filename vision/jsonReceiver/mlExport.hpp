// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MLEXPORT_HPP
#define MLEXPORT_HPP

#include "mlObject.hpp"

#include <QUdpSocket>
#include <QWidget>
#include <QTimer>

class mlExport : public QWidget {
public:
    mlExport();
    void config(const int robot);
    void sendFrame(const QVector<mlObject> allObjects, int camera, int frameId, quint64 mSecs);

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

    bool addObject(mlObject obj);
    void routeCheck();

    QUdpSocket *udpSocket = nullptr;
    QTimer timer;
    camPacketT txPacket;

    QHostAddress groupAddress;
    quint16 port;
    quint8 robot;
};

#endif // MLEXPORT_H
