 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * mainMatchMode.cpp
 *
 * Bridge worldModel to teamplay, only when in match mode.
 * If in test mode, the bridge is open, because robotCLI is expected to stimulate the applicable component.
 *
 *  Created on: Feb 12, 2019
 *      Author: Jan Feitsma
 */

#include "FalconsRtDB2.hpp"

#include "FalconsCommon.h"
#include "tracing.hpp"


int main(int argc, char **argv)
{
    // setup
    int myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    bool matchMode = true;
    auto rtdb = RtDB2Store::getInstance().getRtDB2(myRobotId, teamChar);
    rtdb->put(MATCH_MODE, &matchMode);

    // run
    while (true)
    {
        // wait for new worldModel (which we expect to be on fixed frequency)
        rtdb->waitForPut(ROBOT_STATE);
        
        // trigger teamplay?
        if (matchMode)
        {
            T_TP_HEARTBEAT beat = 0;
            rtdb->put(TP_HEARTBEAT, &beat);
        }
        
        // recalculate matchmode
        int life = 0;
        bool newMatchMode = false;
        int r = rtdb->get(MATCH_MODE, &newMatchMode);
        if (r == 0)
        {
            // behavior: normally, the MATCHMODE key does not exist and the boolean matchMode will always remain true
            // once robotCLI creates the key MATCHMODE and sets it to false, here matchMode becomes false
            if (matchMode != newMatchMode)
            {
                matchMode = newMatchMode;
                tprintf("switching to matchMode=%d", matchMode);
            }
        }
        
    }
    
    return 0;
}
