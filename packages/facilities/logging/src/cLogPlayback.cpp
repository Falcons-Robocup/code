 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogPlayback.cpp
 *
 */

#include <boost/bind.hpp>
#include "ext/cLogPlayback.hpp"

#include "tracing.hpp"

cLogPlayback::cLogPlayback()
{
    _index = 0;
    _rtdb = new RtDB2(0); // agent id is irrelevant when dealing with entire frames at once
}

tLogHeader cLogPlayback::getHeader()
{
    return _header;
}

bool cLogPlayback::step()
{
    TRACE_FUNCTION("");
    _currentFrame = _nextFrame;
    _index++;
    if (!_frameBuffer.getIndex(_index, _nextFrame))
    {
        // this actually stops one frame too early, which doesn't matter much
        return false;
    }
    putFrame(_currentFrame);
    return true;
}

bool cLogPlayback::stepBack()
{
    TRACE_FUNCTION("");
    _nextFrame = _currentFrame;
    _index--;
    if (!_frameBuffer.getIndex(_index, _currentFrame))
    {
        return false;
    }
    putFrame(_currentFrame);
    return true;
}

bool cLogPlayback::seek(rtime t)
{
    // return FALSE in case of improper timestamp, either seeking too early or beyond EOF
    std::ostringstream ss;
    ss << "t=" << t.toStr();
    TRACE_FUNCTION(ss.str().c_str());
    if (t < _header.creation)
    {
        return false;
    }
    // normally given timestamp t is just a bit larger than _t
    // if not, we are apparently scrolling, in which case a linear search is way too expensive
    if ((t < _t) || (double(t - _t) > 0.6))
    {
        // TODO clear current RTDB state
        // seek backwards using binary search
        if (_frameBuffer.timeToIndex(t, _index))
        {
            if (_frameBuffer.getIndex(_index + 1, _nextFrame))
            {
                _frameBuffer.getIndex(_index, _currentFrame);
                putFrame(_currentFrame);
                return true;
            }
        }
        return false;
    }
    // invariant: both _currentFrame and _nextFrame are valid
    // normal iteration: handle a bunch of frames
    rtime tNext = _header.creation + _nextFrame.age;
    while (tNext < t)
    {
        if (!step())
        {
            return false;
        }
        tNext = _header.creation + _nextFrame.age;
    }
    return true;
}

void cLogPlayback::putFrame(tLogFrame const &frame)
{
    TRACE_FUNCTION("");
    // write current frame into RTDB
    rtime t = _header.creation + rtime().fromDouble(frame.age);
    if (_rtdb == NULL)
    {
        throw std::runtime_error("There is no RTDB connection");
    }
    TRACE("putFrame t=%.3f ts=%s size=%d", (double)t, t.toStr().c_str(), (int)frame.data.size());
    _rtdb->putFrameString(frame.data, true);
    // update timestamp administration
    _t = t;
}

