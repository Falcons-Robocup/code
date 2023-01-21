// Copyright 2019-2021 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractStimulator.hpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 *
 * A stimulator is a test utility which feeds an input log file (RTDB stream) into a component, 
 * stimulating it, capturing the resulting RTDB data into an output log file.
 * Any existing outputs in the input log are overwritten.
 * (Visualizer playback can be used to browse any log; python tooling can be used to perform checks.)
 *
 * This is useful when testing the behavior of one component, both regression and progression.
 */

#ifndef CABSTRACTSTIMULATOR_HPP_
#define CABSTRACTSTIMULATOR_HPP_

#include <vector>
#include <string>
#include "cLogPlayback.hpp"
#include "tLogFrame.hpp"

class cAbstractStimulator: public cLogPlayback
{
public:
    cAbstractStimulator(int agentId);
    virtual ~cAbstractStimulator();
    
    // client functions
    virtual bool checkFrame(tLogFrame const &frame) = 0;
    virtual void tick(rtime const &t) = 0;
    
    // process the log (or subset, if applicable)
    void run();
    
    // configuration
    void setVerbosity(int v);
    void add_overruled_key(std::string key);
    void add_deleted_key(std::string key);

private:
    // recalculate the frame, keeping timestamps consistent 
    bool mergeStimFrame(tLogFrame &frame, tLogFrame const &newFrame, rtime const &t);
    RtDB2Frame convertFrameLog2Rtdb(tLogFrame const &frame);
    tLogFrame convertFrameRtdb2Log(float age, std::vector<RtDB2FrameItem> const &items);
    void remove_keys_from_frame(RtDB2Frame& frame, std::vector<std::string> keys);
    void remove_keys_from_frame(tLogFrame& frame, std::vector<std::string> keys);

protected:
    std::string              _inputFile;
    std::string              _outputFile;
    std::string              _component = "<unknown>";
    int                      _agentId = -1;
    rtime                    _tStart;
    rtime                    _tEnd;
    int                      _verbosity = 1;
    std::vector<std::string> _overruled_keys;
    std::vector<std::string> _deleted_keys;
};

#endif

