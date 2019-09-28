 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBInputKeeperFrameAdapter.cpp
 *
 *  Created on: Mar 28, 2019
 *      Author: Jan Feitsma
 */

#include "int/adapters/RTDBInputKeeperFrameAdapter.hpp"

#include <exception>

#include "FalconsCommon.h"
#include <cDiagnostics.hpp>
#include "tracing.hpp"

RTDBInputKeeperFrameAdapter::RTDBInputKeeperFrameAdapter(KeeperFrame& kf)
    : _keeperFrame(kf)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

RTDBInputKeeperFrameAdapter::~RTDBInputKeeperFrameAdapter()
{
    TRACE(">");
    TRACE("<");
}

void RTDBInputKeeperFrameAdapter::loop()
{
    while (true)
    {
        _rtdb->waitForPut(KEEPERFRAME_SETPOINT);
        process();
    }
}

void RTDBInputKeeperFrameAdapter::process()
{
    TRACE_FUNCTION("");
    T_KEEPERFRAME_SETPOINT keeperFrameSetpoint;
    int ageMs = 0, r = 0;

    r = _rtdb->get(KEEPERFRAME_SETPOINT, &keeperFrameSetpoint, ageMs, _myRobotId);
    if ((r == RTDB2_SUCCESS) && (ageMs < 100))
    {
        tprintf("received new keeperFrame setpoint: %s", enum2str(keeperFrameSetpoint));
        TRACE_INFO_TIMEOUT(1.0, "extending keeperFrame %s", enum2str(keeperFrameSetpoint));
        
        switch (keeperFrameSetpoint)
        {
            case keeperFrameSetpointEnum::NONE:
                break;
            case keeperFrameSetpointEnum::LEFT:
                _keeperFrame.setKeeperFrameLeft();
                break;
            case keeperFrameSetpointEnum::RIGHT:
                _keeperFrame.setKeeperFrameRight();
                break;
            case keeperFrameSetpointEnum::UP:
                _keeperFrame.setKeeperFrameUp();
                break;
            case keeperFrameSetpointEnum::UNKNOWN:
            default:
                TRACE_ERROR_TIMEOUT(1.0, "received invalid keeperFrame setpoint");
        }
    }
}

