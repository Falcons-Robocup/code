// Copyright 2019-2020 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cAbstractStimulator.cpp
 *
 *  Created on: Dec 2018
 *      Author: Jan Feitsma
 */


// system includes

// internal includes
#include "cAbstractStimulator.hpp"

// other packages
#include "cLogFileReader.hpp"
#include "cLogFileWriter.hpp"
#include "cLogger.hpp"
#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "FalconsRtDB2.hpp"

cAbstractStimulator::cAbstractStimulator()
{
    systemStdout("rtdbClear"); 
}

cAbstractStimulator::~cAbstractStimulator()
{
}

RtDB2Frame cAbstractStimulator::convertFrameLog2Rtdb(tLogFrame const &frame)
{
    // TODO: refactor so this is not needed, move into rtdb package
    RtDB2Frame result;
    std::string s = frame.data;
    _rtdb->decompress(s);
    result.fromString(s);
    return result;
}

tLogFrame cAbstractStimulator::convertFrameRtdb2Log(float age, std::vector<RtDB2FrameItem> const &items)
{
    // TODO: refactor so this is not needed, move into rtdb package
    RtDB2Frame f;
    f.items = items;
    // serialize and compress
    std::string s = f.toString();
    _rtdb->compress(s);
    tLogFrame result;
    result.age = age;
    result.data = s;
    return result;
}

void cAbstractStimulator::remove_keys_from_frame(RtDB2Frame& frame, std::vector<std::string> keys)
{
    if(!keys.empty())
    {
        for (auto it = frame.items.begin(); it != frame.items.end();)
        {
            if(std::find(keys.begin(), keys.end(), it->key) != keys.end())
            {
                it = frame.items.erase(it);
            }
            else
            {
                it++;
            }
        }
    }
}

void cAbstractStimulator::remove_keys_from_frame(tLogFrame& frame, std::vector<std::string> keys)
{
    if(!keys.empty())
    {
        RtDB2Frame rtdb_frame = convertFrameLog2Rtdb(frame);

        remove_keys_from_frame(rtdb_frame, keys);

        frame = convertFrameRtdb2Log(frame.age, rtdb_frame.items);
    }
}

bool cAbstractStimulator::mergeStimFrame(tLogFrame &frame, tLogFrame const &newFrame, rtime const &t)
{
    // convert tLogFrame (which is serialized and compressed) to RtDB2Frame (a bunch of items)
    // TODO: refactor so this is not needed, move into rtdb package
    RtDB2Frame f1 = convertFrameLog2Rtdb(frame);
    RtDB2Frame f2 = convertFrameLog2Rtdb(newFrame);

    // Remove keys from new frame that should be overruled
    remove_keys_from_frame(f2, _overruled_keys);

    // merge frames
    std::vector<RtDB2FrameItem> newItems = f2.items;
    for (auto itOld = f1.items.begin(); itOld != f1.items.end(); ++itOld)
    {
        bool replaced = false;
        for (auto itNew = f2.items.begin(); itNew != f2.items.end(); ++itNew)
        {
            if ((itOld->agent == itNew->agent) && (itOld->key == itNew->key))
            {
                // item has been replaced no need to add
                replaced = true;
                break;
            }
        }

        if(!replaced)
        {
            newItems.push_back(*itOld);
        }
    }
    
    // convert result
    frame = convertFrameRtdb2Log(frame.age, newItems);

    return true;
}

void cAbstractStimulator::run()
{
    // initialize
    rtime stimStart = rtime::now();
    cLogger logger;
    cLogFileReader inputLog(_inputFile);
    if ((_outputFile.size() == 0) || (_outputFile == "auto"))
    {
        _outputFile = logger.createFileName();
    }
    if (_verbosity >= 1)
    {
        printf("starting %s stimulation of agent %d to file %s ...\n", _component.c_str(), _agentId, _outputFile.c_str());
    }
    cLogFileWriter outputLog(_outputFile);
    _header = inputLog.getHeader();
    rtime t0 = _header.creation;
    _tStart = t0;
    _tEnd = _tStart + _header.duration; // TODO isolate timestamp setting, as client should be allowed to override
    outputLog.writeHeader(_header);
    
    // iterate over input log entries
    tLogFrame frame;
    int numFramesWritten = 0;
    rtime t = t0;

    while (inputLog.getFrame(frame))
    {
        // check against time boundaries
        float age = frame.age;
        t = t0 + age;
        
        if (t >= _tStart)
        {            
            // remove selected keys from existing frame
            remove_keys_from_frame(frame, _deleted_keys);            

            // write to RTDB, preserving original timestamps
            putFrame(frame);
            if (_verbosity >= 4)
            {
                printf("  wrote frame into RTDB   : age=%.3fs, size=%d, t=%s\n", age, (int)frame.data.size(), t.toStr().c_str());
            }
            
            //if (_verbosity >= 3)
            //{
            //    printf("  recalculating frame     : agent=%d, shared=%d, age=%.3fs, t=%s", frame.agent, frame.shared, age, t.toStr().c_str());
            //}
            
            // stimulate the component, so it can recalculate its outputs based on inputs in current frame
            tick(t); 
            
            // make new frame by taking the original one and replacing relevant content with new data
            tLogFrame newFrame;
            bool r = logger.makeFrame(newFrame, t);
            if (r == false)
            {
                fprintf(stderr, "failed to create frame, age=%.3fs\n", age); fflush(stderr);
                // TODO exceptions
                break;
            }
            r = mergeStimFrame(frame, newFrame, t);
            if (r == false)
            {
                fprintf(stderr, "failed to merge frame, age=%.3fs\n", age); fflush(stderr);
                // TODO exceptions
                break;
            }

            // write frame to log
            outputLog.writeFrame(frame);
            numFramesWritten++;
            if (_verbosity >= 4)
            {
                printf("  wrote frame into logfile: age=%.3fs, size=%d, t=%s\n", age, (int)frame.data.size(), t.toStr().c_str());
            }
            
        } // otherwise skip leading frames
    }

    // finish
    if (_verbosity >= 1)
    {
        printf("log file written: %s\n", _outputFile.c_str());
        if (_verbosity >= 2)
        {
            double elapsed = rtime::now() - stimStart;
            printf("stimulation took %.1fs\n", elapsed);
            printf("number of frames written: %d\n", numFramesWritten);
        }
    }
}

void cAbstractStimulator::setVerbosity(int v)
{
    _verbosity = v;
}

void cAbstractStimulator::add_overruled_key(std::string key)
{
    _overruled_keys.push_back(key);
}

void cAbstractStimulator::add_deleted_key(std::string key)
{
    _deleted_keys.push_back(key);
}




