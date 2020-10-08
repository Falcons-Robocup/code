 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cDbSync.cpp
 *
 *  Created on: Aug 19, 2018
 *      Author: Jan Feitsma
 */


#include <boost/bind.hpp>

#include "ext/cDbSync.hpp"

#include "tracing.hpp"

// TODO make arguments, relay from main()
#define RTDB2_STORAGE_PRODUCTION ("/tmp/rtdb2_storage")
#define RTDB2_STORAGE_PLAYBACK ("/tmp/rtdb2_playback")

cDbSync::cDbSync(int frequency)
    : _frequency(frequency)
{
    // use agent id 0 - it does not matter since we deal with entire frames
    _src = new RtDB2(0, RTDB2_STORAGE_PRODUCTION);
    _tgt = new RtDB2(0, RTDB2_STORAGE_PLAYBACK);
}

cDbSync::~cDbSync()
{
}

bool cDbSync::tick()
{
    TRACE("cDbSync::tick start");
    // TODO: check playback mode: are we allowed to live sync?
    // sync the databases, all agents contained in a frame
    RtDB2FrameSelection frameSelection;
    frameSelection.local = true;
    frameSelection.shared = true;
    std::string buffer;
    int r = _src->getFrameString(buffer, frameSelection, -1);
    bool is_empty = (buffer.size() == 1);
    if (r == RTDB2_SUCCESS)
    {
        if (!is_empty)
        {
            _tgt->putFrameString(buffer);
        }
    }
    TRACE("cDbSync::tick end");
    return true;
}

void cDbSync::run()
{
    rtime::loop(_frequency, boost::bind(&cDbSync::tick, this));
}

