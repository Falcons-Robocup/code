// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include "imageTools.hpp"
#include "mjpgSender.hpp"

#include <QObject>
#include <QUdpSocket>
#include <QWidget>
#include <QGraphicsView>
#include <QGraphicsScene>

class receiver : public QWidget
{
    Q_OBJECT
public:
    explicit receiver(QWidget *parent = nullptr, QWidget *window = nullptr, QGraphicsView *camView = nullptr, QGraphicsScene *camScene = nullptr, mjpgSender *mjpgSend = nullptr);
    void setEnable(bool value);
    void toggleShowUpdate();

private slots:
    void processPendingDatagrams();

private:
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
    camPacketT rxPacket;

    void imageUse();
    void routeCheck();

    QUdpSocket *udpSocket = nullptr;
    QHostAddress groupAddress;
    QByteArray data;
    mjpgSender *mjpgSend = nullptr;

    QByteArray jpegData;
    uint8_t cntPrevious;
    bool cntPreviousActive;

    imageTools *imag = nullptr;

    uint32_t frameCounter;
    int packetsReceived;

    void dataProcess();

    bool enabled;
    bool showUpdate;
};

#endif // RECEIVER_HPP

