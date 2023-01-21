// Copyright 2018-2022 Jan Feitsma (Falcons)
// SPDX-License-Identifier: Apache-2.0
/*
 * cLogger.cpp
 *
 *  Created on: Aug 11, 2018
 *      Author: Jan Feitsma
 */


#include <unistd.h>
#include <limits.h>
#include <boost/bind.hpp>

#include "ext/cLogger.hpp"
#include "ext/tLogHeader.hpp"
#include "ext/tLogFrame.hpp"

#include "falconsCommon.hpp"
#include "tracing.hpp"
#include "ftime.hpp"

#include "pstream/pstream.h"

cLogger::cLogger()
    : _frequency(10)
{
    _t0 = ftime::now();
    _te = _t0;
    _rtdb = new FalconsRTDB(getRobotNumber());
    _header.compression = true; // TODO should not hardcode ...
}
    
cLogger::~cLogger()
{
    if (_logFile != NULL)
    {
        _logFile->writeHeader(_header);
    }
}

void cLogger::setFrequency(int frequency)
{
    _frequency = frequency;
}

std::string cLogger::createFileName()
{
    char *envc = getenv("TURTLE5K_ROBOTNUMBER");
    std::string extraStr = "";
    if (envc != NULL)
    {
        std::string robotIdStr(envc);
        if (robotIdStr == "0")
        {
            extraStr = "_coach";
        }
        else if (robotIdStr.size())
        {
            extraStr = "_r" + robotIdStr;
        }
    }
    return "/var/tmp/" + _t0.toStrDate() + extraStr + ".rdl";
}

void cLogger::writeToFile(std::string filename)
{
    // guess filename if not given
    if ((filename.size() == 0) || (filename == "auto"))
    {
        filename = createFileName();
        printf("writing to log file: %s\n", filename.c_str());
    }
    // prepare the file and write header
    _logFile = new cLogFileWriter(filename);
    _header.filename = filename;
    _header.frequency = _frequency;
    _header.compression = true; // TODO should not hardcode ...
    _header.creation = _t0;
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    _header.hostname = hostname;

    redi::ipstream proc_descr_code("cd $FALCONS_CODE_PATH; git describe --tags --long");
    std::getline(proc_descr_code.out(), _header.commit_code);
    redi::ipstream proc_descr_tp_data("cd $FALCONS_TPDATA_PATH; git describe --tags --long");
    std::getline(proc_descr_tp_data.out(), _header.commit_teamplay_data);

    _logFile->writeHeader(_header);
}

bool cLogger::makeFrame(tLogFrame &frame, rtime const &t)
{
    // query RTDB
    auto db = _rtdb;
    if (db == NULL)
    {
        return false;
    }
    RtDB2FrameSelection frameSelection;
    frameSelection.local = true;
    frameSelection.shared = true;
    frameSelection.unique = true;
    std::string buffer;
    // unlike comm2, we should NEVER subsample! (argument -1)
    int r = db->getFrameString(buffer, frameSelection, -1);
    if (r != RTDB2_SUCCESS)
    {
        return false;
    }
    // tricky business: if compression is not used, then for some reason empty frames have size 1
    // if compression, but then non-existent data seems to compress to 10 bytes
    // let's assume there is no other way to construct buffer of size 10 (!!)
    // maybe it would be better that get_batch returns a nonzero code when buffer is empty?
    bool is_empty = (buffer.size() == (_header.compression ? 10 : 1));
    if (is_empty)
    {
        return false;
    }
    // construct frame struct
    _te = t;
    frame.age = double(t - _t0);
    frame.data = buffer;
    return true;
}

bool cLogger::tickNow()
{
    return tick(ftime::now());
}

bool cLogger::tick(rtime const &t)
{
    if (_logFile == NULL)
    {
        return false;
    }
    tLogFrame frame;
    if (makeFrame(frame, t))
    {
        _logFile->writeFrame(frame);
    }
    return true;
}

void cLogger::monitor()
{
    rtime::loop(_frequency, boost::bind(&cLogger::tickNow, this));
}


