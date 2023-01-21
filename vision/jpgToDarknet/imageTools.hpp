// Copyright 2020-2021 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef IMAGE_TOOLS_HPP
#define IMAGE_TOOLS_HPP

#include "mjpgSender.hpp"
#include "detector.hpp"

#include <QByteArray>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QImage>
#include <QQueue>
#include <QString>
#include <QWidget>

class imageTools
{
public:
    imageTools(QWidget *window = nullptr, QGraphicsView *camView = nullptr, QGraphicsScene *camScene = nullptr, mjpgSender *mjpgSend = nullptr);
    void calcFps();
    void rotate(QByteArray &jpegData);
    void save(QByteArray jpegData, QString filename);
    void send(QByteArray jpegData, int frameCounter = 0);
    void setSendSocket(QTcpSocket *sendSocket = nullptr);
    void show(const QByteArray jpegData, const QString description = "", const frame_t objects = {});

private:
    QColor ballColor, obstacleColor, humanColor, outsideColor, goalPostColor;

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

#endif // IMAGE_TOOLS_HPP
