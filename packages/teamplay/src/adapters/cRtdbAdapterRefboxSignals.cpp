 /*** 
 2014 - 2019 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * cRtdbAdapterRefboxSignals.cpp
 *
 *  Created on: Oct 21, 2018
 *      Author: Jan Feitsma
 */

/* Own include */
#include "ext/cRtdbAdapterRefboxSignals.hpp"

/* Other teamplay includes */
#include "int/gameStateManager.hpp"

/* Falcons includes */
#include "FalconsCommon.h" //getTeamChar()
#include "tracing.hpp"

using namespace std;

cRtdbAdapterRefboxSignals::cRtdbAdapterRefboxSignals()
{
    try
    {
        // setup RTDB connection, reading state from coach (id 0)
        _rtdb = RtDB2Store::getInstance().getRtDB2(0, getTeamChar());
    }
    catch (exception &e)
    {
        //TODO TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}

void cRtdbAdapterRefboxSignals::update()
{
    try
    {
        int r = 0;
        int life = 0;
        if (_rtdb != NULL)
        {
            T_MATCH_STATE m;
            r = _rtdb->get(MATCH_STATE, &m, life, 0);
            if (r == RTDB2_SUCCESS)
            {
                std::string dataAsString = m.lastRefboxCommand;
                if (dataAsString.size() > 6)
                {
                    // e.g. "COMM_KICKOFF_OWN"
                    // Remove "COMM_" from string.
                    std::string refboxSig = dataAsString.substr(5, std::string::npos);
                    TRACE("command: %s", refboxSig.c_str());
                    // Convert string to refbox signal enum
                    refboxSignalEnum refboxSignal = refboxSignalMapping[refboxSig];
                    // Store
                    teamplay::gameStateManager::getInstance().refBoxSignalReceived(refboxSignal);
                }
            }
        }
    }
    catch (exception &e)
    {
        //TODO TRACE_ERROR("Caught exception: %s", e.what());
        std::cout << "Caught exception: " << e.what() << std::endl;
        throw std::runtime_error(std::string("Linked to: ") + e.what());
    }
}
