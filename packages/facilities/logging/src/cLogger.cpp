 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
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

#include "FalconsCommon.h"
#include "tracing.hpp"


cLogger::cLogger()
    : _frequency(10)
{
    _t0 = rtime::now();
    _te = _t0;
    _rtdb = new RtDB2(0); // agent id is irrelevant when getting entire frames at once
    _header.compression = true; // TODO should not hardcode ...
}
    
cLogger::~cLogger()
{
    // update header with duration
    _header.duration = double(_te - _t0);
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
    _header.duration = double(_te - _t0);
    char hostname[HOST_NAME_MAX];
    gethostname(hostname, HOST_NAME_MAX);
    _header.hostname = hostname;
    _logFile->writeHeader(_header);
}

bool cLogger::makeFrame(tLogFrame &frame)
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
    rtime t = rtime::now();
    _te = t;
    frame.age = double(t - _t0);
    frame.data = buffer;
    return true;
}

bool cLogger::tick()
{
    if (_logFile == NULL)
    {
        return false;
    }
    tLogFrame frame;
    if (makeFrame(frame))
    {
        _logFile->writeFrame(frame);
    }
    return true;
}

void cLogger::monitor()
{
    rtime::loop(_frequency, boost::bind(&cLogger::tick, this));
}


