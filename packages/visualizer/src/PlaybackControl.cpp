// Copyright 2018-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * PlaybackControl.cpp
 *
 *  Created on: August 28, 2018
 *      Author: Jan Feitsma
 */


#include "int/PlaybackControl.h"
#include "int/widgets/Playback/PlaybackWidget.h"

#include "tracing.hpp"
#include "ftime.hpp"

PlaybackControl::PlaybackControl(bool fileMode, std::string const &filename)
{
    TRACE("> fileMode=%d", fileMode);
    _fileMode = fileMode;
    if (!_fileMode)
    {
        // live visualization mode
        // it must also be possible to pause and scroll back, for instance to
        // answer 'hey what just happened there?!' without having to halt visualizer and load replay
        // if live, then data is visualized as it comes in (slider at the right end)
        // otherwise, playback determines what must be drawn
        //TODO _playback = new cLogMemPlayback();
        //TODO _dbSync = new cDbSync();
        _isLive = true;
        _isPaused = false;
        _tStart = ftime::now();
        _tEnd = ftime::now() + 1.0; // to be updated live
    }
    else
    {
        // file playback mode
        _playback = new cLogFilePlayback(filename);
        // a thread might start to load the file entirely into memory
        auto header = _playback->getHeader();
        _tStart = header.creation;
        TRACE("_tStart=%s", _tStart.toStr().c_str());
        _tEnd = _tStart + _playback->getDuration();
        TRACE("_tEnd=%s", _tEnd.toStr().c_str());
        _isLive = false;
        _isPaused = false;
    }
    _tCurrent = _tStart;
    _frequency = 30.0;
    _tStep = 1.0 / _frequency;
    // start worker 'thread'
    _timer = new QTimer();
    QObject::connect(_timer, &QTimer::timeout, this, &PlaybackControl::tick);
    _timer->start(int(1000.0 * 1.0 / _frequency)); // number in milliseconds
    TRACE("<");
}

PlaybackControl::~PlaybackControl()
{
    if (_dbSync != NULL)
    {
        delete _dbSync;
    }
    if (_playback != NULL)
    {
        delete _playback;
    }
}

void PlaybackControl::registerWidget(PlaybackWidget *widget)
{
    TRACE_FUNCTION("");
    _widget = widget;
}

void PlaybackControl::stepForward()
{
    TRACE_FUNCTION("");
    if (_isPaused)
    {
        _playback->step();
    }
}

void PlaybackControl::stepBack()
{
    TRACE_FUNCTION("");
    if (_isPaused)
    {
        _playback->stepBack();
    }
}

void PlaybackControl::tick()
{
    TRACE_FUNCTION("");
    if (_isLive)
    {
        if (_dbSync != NULL)
        {
            _dbSync->tick();
            _tEnd = ftime::now();
        }
    }
    else
    {
        if (!_isPaused)
        {
            _tCurrent += _tStep;
            bool seekResult = _playback->seek(_tCurrent);
            if (!seekResult)
            {
                TRACE("EOF, _tCurrent=%s", _tCurrent.toStr().c_str());
            }
            // update slider
            if (_widget != NULL)
            {
                int v = round(double(_tCurrent - _tStart) / double(_tEnd - _tStart) * double(SLIDER_RESOLUTION));
                TRACE("v=%d", v);
                _widget->setSliderValue(v);
            }
        }
    }
}

void PlaybackControl::rseek(float v)
{
    std::ostringstream ss;
    ss << "v=" << v << " _isLive=" << _isLive;
    TRACE_FUNCTION(ss.str().c_str());
    if ((v > 0.995) && _isLive)
    {
        _isLive = true;
    }
    else
    {
        _isLive = false;
        rtime tNew = _tStart + double(_tEnd - _tStart) * v;
        // for continuity, don't modify timestamp administration unless really needed
        double delta = tNew - _tCurrent;
        if (delta > 0.1 || delta < 0.0)
        {
            _tCurrent = tNew;
            TRACE("scrolling to %s", _tCurrent.toStr().c_str());
            // TODO clear current visualizer state
            bool seekResult = _playback->seek(_tCurrent);
            if (!seekResult)
            {
                TRACE("this is weird - how can one scroll to an invalid timestamp? slider (size) bug? _tCurrent=%s", _tCurrent.toStr().c_str());
            }
        }
    }
}

void PlaybackControl::togglePause()
{
    std::ostringstream ss;
    ss << "paused=" << _isPaused;
    TRACE_FUNCTION(ss.str().c_str());
    _isPaused = !_isPaused;
}

bool PlaybackControl::isPaused()
{
    return _isPaused;
}

void PlaybackControl::setSpeed(float speed)
{
    _tStep = speed / _frequency;
}

