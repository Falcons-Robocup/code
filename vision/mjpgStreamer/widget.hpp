// Copyright 2020 Andre Pool
// SPDX-License-Identifier: Apache-2.0

#ifndef WIDGET_HPP
#define WIDGET_HPP

#include "receiver.hpp"
#include "fileReader.hpp"

#include <QWidget>
#include <QSettings>

QT_BEGIN_NAMESPACE
namespace Ui { class Widget; }
QT_END_NAMESPACE

class Widget : public QWidget
{
    Q_OBJECT
public:
    Widget(QWidget *parent = nullptr);
    void camViewSize();

private slots:
    void on_modeButton_clicked();
    void on_exitButton_clicked();
    void on_fastReverseButton_clicked();
    void on_reverseButton_clicked();
    void on_pauseButton_clicked();
    void on_forwardButton_clicked();
    void on_fastForwardButton_clicked();
    void on_imageSlider_sliderMoved(int position);
    void on_imageSlider_valueChanged(int value);

    void on_showButton_clicked();

    void on_revereseOneButton_clicked();

    void on_forwardOneButton_clicked();

private:
    void exitAfterSec(int seconds);
    void exitTimerOccured();
    void moveEvent(QMoveEvent *);
    void resizeEvent(QResizeEvent*);
    void saveWindowGeometry();
    void setGetConfig();

    receiver *recv = nullptr;
    fileReader *fileRead = nullptr;

    QSettings *settings = nullptr;

    Ui::Widget *ui;
    bool fileMode;
    bool showUpdate;
};

#endif // WIDGET_HPP
