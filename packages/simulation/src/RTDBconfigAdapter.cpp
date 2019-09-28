 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * RTDBConfigAdapter.cpp
 *
 *  Created on: Aug 17, 2019
 *      Author: Coen Tempelaars
 */

#include "int/RTDBconfigAdapter.hpp"
#include "int/RTDBaccess.hpp"

#include <stdexcept>

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"


std::string RTDBConfigAdapter::getArbiter() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    int ageMs = 0;
    auto rtdbConnection = getRTDBConnection();
    auto r = rtdbConnection->get("CONFIG_SIMULATION", &config, ageMs, 0);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting CONFIG_SIMULATION from RtDB");
    }
    tprintf("get CONFIG_SIMULATION arbiter=%s", config.arbiter.c_str());
    return config.arbiter;
}

int RTDBConfigAdapter::getSize(const TeamID teamID) const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    int teamSize = 0;
    int ageMs = 0;
    auto rtdbConnection = getRTDBConnection();
    auto r = rtdbConnection->get("CONFIG_SIMULATION", &config, ageMs, 0);
    if (r != RTDB2_SUCCESS)
    {
        throw std::runtime_error("Error getting CONFIG_SIMULATION from RtDB");
    }

    if (teamID == TeamID::A)
    {
        teamSize = config.sizeTeamA;
    }
    else
    {
        teamSize = config.sizeTeamB;
    }

    tprintf("get CONFIG_SIMULATION teamSize=%d", teamSize);
    return teamSize;
}
