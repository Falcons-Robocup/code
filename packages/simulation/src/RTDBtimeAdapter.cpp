 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBTimeAdapter.cpp
 *
 *  Created on: June 07, 2020
 *      Author: Erik Kouters
 */

#include "int/RTDBtimeAdapter.hpp"
#include "int/RTDBaccess.hpp"

#include <stdexcept>

#include "tracing.hpp"

void RTDBTimeAdapter::waitForSimulationTick() const
{
    TRACE_FUNCTION("");
    
    auto rtdbConnection = getRTDBConnection();
    rtdbConnection->waitForPut(SIMULATION_TICK);
}

void RTDBTimeAdapter::publishSimulationTick() const
{
    TRACE_FUNCTION("");
    
    auto rtdbConnection = getRTDBConnection();
    T_SIMULATION_TICK tick = 0;
    rtdbConnection->put(SIMULATION_TICK, &tick);
}

void RTDBTimeAdapter::publishSimulationTime (const rtime& simTime) const
{
    TRACE_FUNCTION("");
    T_SIMULATION_TIME rtdbSimTime = simTime.toDouble();

    auto rtdbConnection = getRTDBConnection();

    tprintf("put SIMULATION_TIME = %f", rtdbSimTime);
    auto r = rtdbConnection->put(SIMULATION_TIME, &rtdbSimTime);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error writing SIMULATION_TIME to RtDB");
    }
}
