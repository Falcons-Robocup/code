 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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
#include "FalconsCommon.h"
#include "tracing.hpp"

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

bool cAbstractStimulator::mergeStimFrame(tLogFrame &frame, tLogFrame const &newFrame, rtime const &t)
{
    // convert tLogFrame (which is serialized and compressed) to RtDB2Frame (a bunch of items)
    // TODO: refactor so this is not needed, move into rtdb package
    RtDB2Frame f1 = convertFrameLog2Rtdb(frame);
    RtDB2Frame f2 = convertFrameLog2Rtdb(newFrame);
    std::vector<RtDB2FrameItem> newItems = f1.items;
    int countSame = 0, countChanged = 0, countAdd = 0;
    for (auto itNew = f2.items.begin(); itNew != f2.items.end(); ++itNew)
    {
        auto newItem = *itNew;
        // check if item in newFrame is also present in frame
        bool found = false;
        auto itOld = f1.items.begin();
        for (; !found && (itOld != f1.items.end()); ++itOld)
        {
            if ((itOld->agent == itNew->agent) && (itOld->key == itNew->key))
            {
                found = true;
                break;
                // we will refer to itOld
            }
        }
        if (found)
        {
            // item in newFrame is also present in frame -> check if it is the same
            // (this normally holds for most items -- TODO count and show some stats?)
            bool same = (itOld->data == itNew->data);
            if (same)
            {
                //tprintf("same    agent=%d key='%s'", itNew->agent, itNew->key.c_str());
                countSame++;
                // nothing to do, item was not touched by new software, it can remain as is in the frame
            }
            else
            {
                // stimulated software produced new data, so overwrite
                //int n = (int)itNew->data.size();
                //tprintf("changed agent=%d key='%s' n=%d", itNew->agent, itNew->key.c_str(), n);
                countChanged++;
                newItem.timestamp = itOld->timestamp; // leave original timestamp intact
                newItems.push_back(newItem);
            }
        }
        else
        {
            //tprintf("add     agent=%d key='%s'", itNew->agent, itNew->key.c_str());
            countAdd++;
            // TODO: is this appropriate? newest logger call might pick up items which would normally be outdated ... how to properly solve this?
            newItem.timestamp = t;
            newItems.push_back(newItem);
        }
    }
    
    // convert result
    //tprintf("age=%.3fs counts: #new=%d, #same=%d, #change=%d, #add=%d", frame.age, (int)newItems.size(), countSame, countChanged, countAdd);
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
        if (t > _tEnd)
        {
            break;
        }
        if (t >= _tStart)
        {
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
            bool r = logger.makeFrame(newFrame);
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


