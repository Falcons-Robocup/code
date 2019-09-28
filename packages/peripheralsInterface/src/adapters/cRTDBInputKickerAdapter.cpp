 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputKickerAdapter.cpp
 *
 *  Created on: Sep 23, 2018
 *      Author: Jan Feitsma
 */

#include "int/adapters/cRTDBInputKickerAdapter.hpp"

#include <exception>

#include "FalconsCommon.h"
#include <cDiagnostics.hpp>
#include "tracing.hpp"

cRTDBInputKickerAdapter::cRTDBInputKickerAdapter(Kicker& kicker)
    : kicker(kicker)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId);
    TRACE("<");
}

cRTDBInputKickerAdapter::~cRTDBInputKickerAdapter()
{
    TRACE(">");
    TRACE("<");
}

void cRTDBInputKickerAdapter::waitForKickerSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(KICKER_SETPOINT);
        getKickerSetpoint();
    }
}

void cRTDBInputKickerAdapter::getKickerSetpoint()
{
    TRACE_FUNCTION("");
    T_KICKER_SETPOINT kickerSetpoint;

    _rtdb->get(KICKER_SETPOINT, &kickerSetpoint);

    switch(kickerSetpoint.kickerSetpointType)
    {
        case kickerSetpointTypeEnum::HOME:
        {
            TRACE_SCOPE("HOME_KICKER","");
            kicker.home();
            break;
        }
        case kickerSetpointTypeEnum::SET_HEIGHT:
        {
            std::stringstream str;
            str << "height=" << kickerSetpoint.kickerHeight;
            TRACE_SCOPE("SET_KICKER_HEIGHT", str.str().c_str());

            kicker.setHeight(kickerSetpoint.kickerHeight);
            kicker.move();
            break;
        }
        case kickerSetpointTypeEnum::SHOOT:
        {
            std::stringstream str;
            str << "power=" << kickerSetpoint.kickerPower;
            TRACE_SCOPE("SET_KICKER_POWER", str.str().c_str());

            // only shoot if power > 0
            if (kickerSetpoint.kickerPower > 0)
            {
                kicker.setShootPower(kickerSetpoint.kickerPower);
                kicker.shoot();
            }
            break;
        }
    }

}
