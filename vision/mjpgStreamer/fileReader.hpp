// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef FILEREADER_HPP
#define FILEREADER_HPP

#include "image.hpp"
#include "mjpgSender.hpp"

#include <QByteArray>
#include <QGraphicsScene>
#include <QGraphicsView>
#include <QSlider>
#include <QTimer>
#include <QVector>
#include <QWidget>

class fileReader
{
    Q_ENUMS(playModeType)
public:
    fileReader(QWidget *window = nullptr, QGraphicsView *camView = nullptr, QGraphicsScene *camScene = nullptr, QSlider *slider = nullptr, mjpgSender *mjpgSend = nullptr);
    void config();
    void setEnable(bool value);
    void setFastReverse();
    void setReverse();
    void setOneReverse();
    void setPause();
    void setOneForward();
    void setForward();
    void setFastForward();
    void setSliderMove(int value);
    void setSliderValueChanged();

private:
    void updateIndex();
    void getImage();

    QWidget *window = nullptr;
    QTimer *readTimer = nullptr;
    QTimer *sendTimer = nullptr;
    QSlider *slider = nullptr;
    image *imag = nullptr;
    mjpgSender *mjpgSend = nullptr;

    enum playModeType { fastReverse = 1, reverse, oneReverse, pause, oneForward, forward, fastForward, sliderMove };
    playModeType playMode;

    QVector<QString> allFiles;
    QByteArray jpegData;
    bool enabled;
};

#endif // FILEREADER_HPP
