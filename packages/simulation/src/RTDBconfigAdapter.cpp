 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
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
#include <thread>

#include "FalconsRtDB2.hpp"
#include "tracing.hpp"

RTDBConfigAdapter::RTDBConfigAdapter()
{
    // Fetch configuration from yaml file
    //std::string configFile = determineConfig("simulation");
    _configAdapter = new ConfigRTDBAdapter<T_CONFIG_SIMULATION>(CONFIG_SIMULATION);
    _configAdapter->load();
}

RTDBConfigAdapter::~RTDBConfigAdapter()
{
    delete _configAdapter;
}

std::string RTDBConfigAdapter::getArbiter() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    tprintf("get CONFIG_SIMULATION arbiter=%s", config.arbiter.c_str());
    return config.arbiter;
}

int RTDBConfigAdapter::getSize(const TeamID teamID) const
{
    TRACE_FUNCTION("");

    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);

    int teamSize = 0;
    if (teamID == TeamID::A)
    {
        teamSize = config.sizeTeamA;
        tprintf("get CONFIG_SIMULATION teamID=A teamSize=%d", teamSize);
    }
    else
    {
        teamSize = config.sizeTeamB;
        tprintf("get CONFIG_SIMULATION teamID=B teamSize=%d", teamSize);
    }

    return teamSize;
}

int RTDBConfigAdapter::getTickFrequency() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    //tprintf("get CONFIG_SIMULATION tickFrequency=%d", config.tick_frequency);
    return config.tick_frequency;
}

int RTDBConfigAdapter::getStepSizeMs() const
{
    TRACE_FUNCTION("");
    T_CONFIG_SIMULATION config;
    _configAdapter->get(config);
    tprintf("get CONFIG_SIMULATION stepSize=%d", config.tick_stepsize_ms);
    return config.tick_stepsize_ms;
}
