// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef MJPGSENDER_HPP
#define MJPGSENDER_HPP

#include <QByteArray>
#include <QMutex>
#include <QTcpServer>
#include <QTcpSocket>
#include <QWidget>


class mjpgSender : public QWidget
{
    Q_OBJECT

public:
    mjpgSender(QWidget *parent = nullptr, QWidget *window = nullptr);
    void send(QByteArray jpegData, int frameIndex);
    void setDirectMode(bool value);

public slots:
    void clientConnected();

private:
    void doSend();
    QTcpServer *server = nullptr;
    QTcpSocket *socket = nullptr;
    bool enabled;
    QTimer *sendTimer = nullptr;
    QMutex mutex;
    QByteArray jpegData;
    int frameCounter;
    bool directMode;
};

#endif // MJPGSENDER_HPP
