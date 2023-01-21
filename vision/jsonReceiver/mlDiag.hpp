// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MLDIAG_HPP
#define MLDIAG_HPP

#include "mlObject.hpp"

#include <QObject>
#include <QUdpSocket>
#include <QWidget>
#include <QLabel>

class mlDiag : public QWidget {
public:
    explicit mlDiag();
    void config(const int robot);
    void sendFrame(const QVector<mlObject> allObjects, int camera, int frameId, quint64 mSecs);

private:
    QUdpSocket *udpSocket = nullptr;
    QHostAddress groupAddress;
    quint16 port;
    QByteArray datagram;
    quint8 robot;

    bool addObject(mlObject obj);
    void sendStats(int camera, int frameIdValue, quint64 mSecs);
    void routeCheck();

};

#endif // MLDIAG_HPP
