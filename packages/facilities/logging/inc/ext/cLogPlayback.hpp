 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cLogPlayback.hpp
 *
 * Base class for playback. Intended to split in-memory use case (live visualization) from post-mortem file-based playback.
 *
 *  Created on: Sep 2, 2018
 *      Author: Jan Feitsma
 */

#ifndef CLOGPLAYBACK_HPP_
#define CLOGPLAYBACK_HPP_


#include <utility>
#include "tLogHeader.hpp"
#include "tLogFrame.hpp"
#include "cFrameBuffer.hpp"
#include "cDbConnection.hpp"


class cLogPlayback
{
public:
    cLogPlayback();
    virtual ~cLogPlayback() {};

    tLogHeader getHeader();
    bool step(); // process a single frame, return success
    bool stepBack();
    bool seek(rtime t); // advance/jump to given timestamp
    
protected:
    RtDB2* _rtdb = NULL;
    tLogHeader _header; // header of this log
    rtime _t; // current timestamp
    tLogFrame _currentFrame;
    tLogFrame _nextFrame;
    cFrameBuffer _frameBuffer; // fast random access, chronologically-ordered
    size_t _index;
    // is required for stepping through time, handling frames until nextFrame is too new
    void putFrame(tLogFrame const &frame);
};

#endif

