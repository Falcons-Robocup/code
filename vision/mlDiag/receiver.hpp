// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef RECEIVER_HPP
#define RECEIVER_HPP

#include "frame.hpp"

#include <QFile>
#include <QHostAddress>
#include <QMutex>
#include <QUdpSocket>
#include <QVector>
#include <QWidget>

class receiver : public QWidget
{
    Q_OBJECT
public:

    explicit receiver(QWidget *parent = nullptr);
    ~receiver();
    QVector<frame> getFrameList(int robot, int camera);

private slots:
    void processPendingDatagrams();

private:
    void dataProcess();
    void routeCheck();

    QUdpSocket *udpSocket;
    QHostAddress groupAddress;
    QFile *recording = nullptr;
    QByteArray data;
    QByteArray objectList[ROBOTS][CAMERAS];
    int objectsPending[ROBOTS][CAMERAS];
    bool objectsPendingCheck[ROBOTS][CAMERAS];
    QVector<frame> frameList[ROBOTS][CAMERAS];

    QString recordingFileName;
    QMutex mutex;
};

#endif // RECEIVER_HPP
