// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDbSync.cpp
 *
 *  Created on: Aug 19, 2018
 *      Author: Jan Feitsma
 */


#include <boost/bind.hpp>

#include "ext/cDbSync.hpp"
#include "ext/cDbConnection.hpp"

#include "tracing.hpp"


cDbSync::cDbSync(int frequency)
    : _frequency(frequency)
{
    // use agent id 0 - it does not matter since we deal with entire frames
    _src = FalconsRTDBStore::getInstance().getFalconsRTDB(0, RTDB_STORAGE_PRODUCTION);
    _tgt = FalconsRTDBStore::getInstance().getFalconsRTDB(0, RTDB_STORAGE_PLAYBACK);
}

cDbSync::~cDbSync()
{
}

bool cDbSync::tick()
{
    TRACE("cDbSync::tick start");
    // TODO: check playback mode: are we allowed to live sync?
    // sync the databases, all agents contained in a frame
    RtDB2FrameSelection frameSelection;
    frameSelection.local = true;
    frameSelection.shared = true;
    std::string buffer;
    int r = _src->getFrameString(buffer, frameSelection, -1);
    bool is_empty = (buffer.size() == 1);
    if (r == RTDB2_SUCCESS)
    {
        if (!is_empty)
        {
            _tgt->putFrameString(buffer);
        }
    }
    TRACE("cDbSync::tick end");
    return true;
}

void cDbSync::run()
{
    rtime::loop(_frequency, boost::bind(&cDbSync::tick, this));
}

