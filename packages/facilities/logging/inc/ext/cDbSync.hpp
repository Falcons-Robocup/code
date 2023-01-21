// Copyright 2018-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cDbSync.hpp
 *
 * Sync one RTDB storage to another.
 * Used on coach to split visualization from production database.
 * The production database resides by default at /tmp/rtdb2_storage. It is used during a game by comm, refbox, and other coach processed.
 * The other database is /tmp/rtdb2_playback, which is used by the visualizer.
 * This allows the visualizer to simply draw the contents of /tmp/rtdb2_playback, even managing it itself (slider) using cPlayBack.
 * During such live pausing / scrolling, this sync utility must pause. 
 *
 *  Created on: Aug 19, 2018
 *      Author: Jan Feitsma
 */

#ifndef CDBSYNC_HPP_
#define CDBSYNC_HPP_


#include "FalconsRTDB.hpp"


class cDbSync
{
  public:
    cDbSync(int frequency = 20);
    ~cDbSync();
    
    void run(); // loop around tick()
    bool tick(); // poked from run
    
  private:
    int _frequency;
    FalconsRTDB *_src = NULL;
    FalconsRTDB *_tgt = NULL;
    
};

#endif

