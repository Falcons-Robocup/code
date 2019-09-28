 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRTDBInputAdapter.cpp
 *
 *  Created on: Dec 27, 2018
 *      Author: Erik Kouters
 */

#include "tracing.hpp"
#include "FalconsCommon.h" //getRobotNumber(), getTeamChar()

#include "int/adapters/cRTDBInputAdapter.hpp"

cRTDBInputAdapter::cRTDBInputAdapter(cShootPlanner *shootPlanner)
{
    TRACE(">");
    _myRobotId = getRobotNumber();
    auto teamChar = getTeamChar();
    _rtdb = RtDB2Store::getInstance().getRtDB2(_myRobotId, teamChar);
    _shootPlanner = shootPlanner;
    TRACE("<");
}

cRTDBInputAdapter::~cRTDBInputAdapter()
{
}

void cRTDBInputAdapter::waitForShootSetpoint()
{
    while (true)
    {
        _rtdb->waitForPut(SHOOT_SETPOINT);
        getShootSetpoint();
    }
}

void cRTDBInputAdapter::getShootSetpoint()
{
    TRACE_FUNCTION("");
    T_SHOOT_SETPOINT shootSetpoint;

    int r = _rtdb->get(SHOOT_SETPOINT, &shootSetpoint);
    
    tprintf("             get SHOOT_SETPOINT phase=%s type=%s pos=[%6.2f, %6.2f, %6.2f]", enum2str(shootSetpoint.shootPhase), enum2str(shootSetpoint.shootType), shootSetpoint.position.x, shootSetpoint.position.y, shootSetpoint.position.z);

    if (r == RTDB2_SUCCESS)
    {
        switch(shootSetpoint.shootPhase)
        {
            case shootPhaseEnum::NONE:
            {
                // do nothing
                break;
            }
            case shootPhaseEnum::PREPARE:
            {
                if (_shootPlanner != NULL)
                {
                    _shootPlanner->prepareForShot(shootSetpoint.position.x, shootSetpoint.position.z, shootSetpoint.shootType);
                }
                break;
            }
            case shootPhaseEnum::SHOOT:
            {
                switch(shootSetpoint.shootType)
                {
                    case shootTypeEnum::PASS:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executePass(shootSetpoint.position.x);
                        }
                        break;
                    }
                    case shootTypeEnum::SHOOT:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executeShot();
                        }
                        break;
                    }
                    case shootTypeEnum::LOB:
                    {
                        if (_shootPlanner != NULL)
                        {
                            _shootPlanner->executeShot();
                        }
                        break;
                    }
                }
                break;
            }
        }
    }
}
