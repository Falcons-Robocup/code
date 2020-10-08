 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogFilePlayback.cpp
 *
 */

#include <boost/bind.hpp>
#include "ext/cLogFilePlayback.hpp"

#include "tracing.hpp"


cLogFilePlayback::cLogFilePlayback(std::string const &filename)
{
    _logFile = new cLogFileReader(filename);
    load(); // TODO: could take a while ... move this into separate thread?
}

cLogFilePlayback::~cLogFilePlayback()
{
}

void cLogFilePlayback::load()
{
    TRACE_FUNCTION("");
    _header = _logFile->getHeader();
    // set current timestamp
    _t = _header.creation;
    TRACE("creation=%s", _t.toStr().c_str());
    // load file into memory
    tLogFrame frame;
    while (_logFile->getFrame(frame))
    {
        rtime t = _header.creation + frame.age;
        _frameBuffer.insert(t, frame);
    }
    _frameBuffer.traceStats();
    // read first frame(s)
    _frameBuffer.getIndex(0, _currentFrame);
    _frameBuffer.getIndex(1, _nextFrame);
    _index = 0;
}

