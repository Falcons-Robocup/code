// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#include "mjpgSender.hpp"
#include "receiver.hpp"
#include "ui_widget.h"
#include "widget.hpp"

#include <QTimer>
#include <QDebug>
#include <QGraphicsScene>

Widget::Widget(QWidget *parent)
    : QWidget(parent)
    , ui(new Ui::Widget)
{
    ui->setupUi(this);
    this->setWindowTitle("mjpg streamer");

    showUpdate = true;

    setGetConfig();

    mjpgSender *mjpgSend = new mjpgSender(parent, this);
    QGraphicsScene *camScene = new QGraphicsScene(this);

    fileRead = new fileReader(this, ui->camView, camScene, ui->imageSlider, mjpgSend); // use only setGeometry from this (part of QWidget)
    fileRead->config();
    recv = new receiver(parent, this, ui->camView, camScene, mjpgSend); // use only setGeometry from this (part of QWidget)

    if( fileMode ) {
        fileRead->setEnable(true);
        recv->setEnable(false);
    } else {
        fileRead->setEnable(false);
        recv->setEnable(true);
    }

    // automatic exit application (for testing)
    // exitAfterSec(10);
}

void Widget::setGetConfig() {
    // manage the main window size
    settings = new QSettings("falcons", "mjpegStreamer");

    // get the saved geometry
    settings->beginGroup("layout");
    int height = settings->value("windowHeight").toInt();
    int width = settings->value("windowWidth").toInt();
    int left = settings->value("windowLeft").toInt();
    int top = settings->value("windowTop").toInt();
    settings->endGroup();

    if( width != 0 ) {
        // use the settings from the stored configuration to position the window
        this->setGeometry(QRect(left, top, width, height));
    }

    settings->beginGroup("config");
    fileMode = settings->value("fileMode").toBool();
    settings->endGroup();   
}

void Widget::exitAfterSec(int seconds) {
    QTimer *exitTimer = new QTimer(this);
    exitTimer->setInterval(seconds * 1000);
    connect( exitTimer, &QTimer::timeout,[=]() { exitTimerOccured(); });
    exitTimer->start();
}

void Widget::exitTimerOccured() {
    qDebug("INFO    the exit timer expired, stop application (expected behavior)");
    QCoreApplication::exit();
}

void Widget::on_modeButton_clicked() {
    fileMode = ! fileMode;

    if( fileMode ) {
        fileRead->setEnable(true);
        recv->setEnable(false);
    } else {
        fileRead->setEnable(false);
        recv->setEnable(true);
    }

    // store preference
    settings->beginGroup("config");
    settings->setValue("fileMode", fileMode);
    settings->endGroup();
}

void Widget::on_exitButton_clicked() {
    qDebug() << "INFO    requested to exit through button";
    QApplication::quit();
}

void Widget::saveWindowGeometry() {
    // get the current geometry
    QRect windowRect = this->geometry();
    // store in config
    settings->beginGroup("layout");
    settings->setValue("windowHeight", windowRect.height());
    settings->setValue("windowWidth", windowRect.width());
    settings->setValue("windowLeft", windowRect.left());
    settings->setValue("windowTop", windowRect.top());
    settings->endGroup();
}

void Widget::resizeEvent(QResizeEvent*) {
    saveWindowGeometry();
}

void Widget::moveEvent(QMoveEvent *) {
    saveWindowGeometry();
}

void Widget::on_fastReverseButton_clicked() {
    fileRead->setFastReverse();
}

void Widget::on_reverseButton_clicked() {
    fileRead->setReverse();
}

void Widget::on_revereseOneButton_clicked() {
    fileRead->setOneReverse();
}

void Widget::on_pauseButton_clicked() {
    fileRead->setPause();
}

void Widget::on_forwardOneButton_clicked() {
    fileRead->setOneForward();
}
void Widget::on_forwardButton_clicked() {
    fileRead->setForward();
}

void Widget::on_fastForwardButton_clicked() {
    fileRead->setFastForward();
}

void Widget::on_imageSlider_sliderMoved(int position) {
    fileRead->setSliderMove(position);
}

void Widget::on_imageSlider_valueChanged(int value) {
    (void)(value);
    fileRead->setSliderValueChanged();
}

void Widget::on_showButton_clicked() {
    recv->toggleShowUpdate();
}

