// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlaybackControl.h
 *
 *  Created on: August 28, 2018
 *      Author: Jan Feitsma
 */

#ifndef PLAYBACKCONTROL_H
#define PLAYBACKCONTROL_H

#include <string>
#include <QtGui>

#include "cLogFilePlayback.hpp"
#include "cDbSync.hpp"
#include "FalconsRTDB.hpp"

#define SLIDER_RESOLUTION 1000.0

class PlaybackWidget; // avoid DoD

class PlaybackControl : public QObject
{
    Q_OBJECT
    
public:
    PlaybackControl(bool fileMode, std::string const &filename);
    ~PlaybackControl();

    void registerWidget(PlaybackWidget *widget);
    void tick();
    void rseek(float v); // relative seek: 0.0 is start, 1.0 is end
    void stepForward();
    void stepBack();
    void togglePause();
    bool isPaused();
    void setSpeed(float speed);
    
private:
    bool _fileMode = false;
    bool _isLive = false;
    bool _isPaused = false;
    rtime _tStart;
    rtime _tEnd;
    rtime _tCurrent;
    rtime _tStep;
    float _frequency;
    cLogPlayback *_playback = NULL;
    cDbSync *_dbSync = NULL;
    QTimer *_timer = NULL;
    PlaybackWidget *_widget = NULL;
};

#endif

