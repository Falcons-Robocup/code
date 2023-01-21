// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef IMAGE_HPP
#define IMAGE_HPP

#include "mjpgSender.hpp"

#include <QByteArray>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QQueue>
#include <QString>
#include <QWidget>

class image
{
public:
    image(QWidget *window = nullptr, QGraphicsView *camView = nullptr, QGraphicsScene *camScene = nullptr, mjpgSender *mjpgSend = nullptr);
    void calcFps();
    void rotate(QByteArray &jpegData);
    void save(QByteArray jpegData, QString filename);
    void send(QByteArray jpegData, int frameCounter = 0);
    void setSendSocket(QTcpSocket *sendSocket = nullptr);
    void show(QByteArray jpegData, QString description = "");

private:
    int borderWidth;
    int borderHeight;
    QQueue<qint64> captureTime;
    QGraphicsScene *camScene = nullptr;
    QGraphicsView *camView = nullptr;
    float fps;
    mjpgSender *mjpgSend = nullptr;
    QFont myFont;
    QWidget *window = nullptr;
};

#endif // IMAGE_HPP
