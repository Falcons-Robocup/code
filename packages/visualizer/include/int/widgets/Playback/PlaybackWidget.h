// Copyright 2018 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlaybackWidget.h
 *
 *  Created on: August 28, 2018
 *      Author: Jan Feitsma
 */

#ifndef PLAYBACKWIDGET_H
#define PLAYBACKWIDGET_H

#include <qslider.h>
#include <qpushbutton.h>
#include <QComboBox>

// Internal:
#include "int/widgets/Widget.h"
#include "int/PlaybackControl.h"

class PlaybackGameSignalSubscriber;

class PlaybackWidget : public QWidget, public Widget<PlaybackGameSignalSubscriber, PlaybackWidget>
{
    Q_OBJECT
    friend PlaybackGameSignalSubscriber;

  public:
    PlaybackWidget(QWidget* parent = 0);
    void registerPlaybackControl(PlaybackControl *pb);
    void setSliderValue(int v);

  private:
    PlaybackControl *_pbControl = NULL;
    QSlider *_slider = NULL;
    QPushButton *_button1 = NULL;
    QComboBox *_dropdown = NULL;
    
  public Q_SLOTS:
    void onSliderValueChange(int v);
    void onButton1Change();
    void onSpeedChange(int idx);
    void stepBack();
    void stepForward();

};

// unused
class PlaybackGameSignalSubscriber : public QObject, public WidgetGameSignalSubscriber<PlaybackWidget>
{
    Q_OBJECT
  public:
    using WidgetGameSignalSubscriber::WidgetGameSignalSubscriber;
    virtual void subscribe(GameSignalAdapter* gameSignalAdapter) override;
};

#endif // PLAYBACKWIDGET_H

