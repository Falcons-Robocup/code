 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cFrameBuffer.cpp
 *
 */


#include "ext/cFrameBuffer.hpp"

#include "tracing.hpp"

cFrameBuffer::cFrameBuffer()
{
    _bytes = 0;
}

cFrameBuffer::~cFrameBuffer()
{
}

void cFrameBuffer::insert(rtime const &t, tLogFrame const &frame)
{
    _indexHash[t] = _array.size();
    _array.push_back(frame);
    // extra, for tracing only ... might be a bit expensive:
    msgpack::sbuffer sbuf;
    msgpack::pack(sbuf, frame);
    _bytes += sbuf.size();
}

bool cFrameBuffer::getIndex(size_t idx, tLogFrame &frame)
{
    if (idx < _array.size())
    {
        frame = _array[idx];
        return true;
    }
    return false;
}

bool cFrameBuffer::timeToIndex(rtime const &t, size_t &idx)
{
    TRACE_FUNCTION("");
    auto it = _indexHash.upper_bound(t);
    if (it == _indexHash.end())
    {
        return false;
    }
    if (it != _indexHash.begin())
    {
        --it;
    }
    idx = it->second;
    TRACE("t=%s idx=%d", t.toStr().c_str(), idx);
    return true;
}

void cFrameBuffer::traceStats()
{
    float duration = _array.rbegin()->age;
    size_t numFrames = _array.size();
    TRACE("indexed %d frames, avg %.1f frames/sec, total size %.2fMB", numFrames, numFrames / duration, _bytes / 1024.0 / 1024.0);
}

