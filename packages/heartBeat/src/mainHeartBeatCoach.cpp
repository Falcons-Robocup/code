 /*** 
 2014 - 2020 ASML Holding N.V. All Rights Reserved. 
 
 NOTICE: 
 
 IP OWNERSHIP All information contained herein is, and remains the property of ASML Holding N.V. The intellectual and technical concepts contained herein are proprietary to ASML Holding N.V. and may be covered by patents or patent applications and are protected by trade secret or copyright law. NON-COMMERCIAL USE Except for non-commercial purposes and with inclusion of this Notice, redistribution and use in source or binary forms, with or without modification, is strictly forbidden, unless prior written permission is obtained from ASML Holding N.V. 
 
 NO WARRANTY ASML EXPRESSLY DISCLAIMS ALL WARRANTIES WHETHER WRITTEN OR ORAL, OR WHETHER EXPRESS, IMPLIED, OR STATUTORY, INCLUDING BUT NOT LIMITED, ANY IMPLIED WARRANTIES OR CONDITIONS OF MERCHANTABILITY, NON-INFRINGEMENT, TITLE OR FITNESS FOR A PARTICULAR PURPOSE. 
 
 NO LIABILITY IN NO EVENT SHALL ASML HAVE ANY LIABILITY FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING WITHOUT LIMITATION ANY LOST DATA, LOST PROFITS OR COSTS OF PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES), HOWEVER CAUSED AND UNDER ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE OR THE EXERCISE OF ANY RIGHTS GRANTED HEREUNDER, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGES 
 ***/ 
 /*
 * main.cpp
 *
 *  Created on: Nov 10, 2015
 *      Author: Tim Kouters
 */

#include <stdexcept>
#include <chrono>
#include <exception>
#include <functional>
#include <thread>

#include "ConfigRTDBAdapter.hpp"
#include "cDiagnostics.hpp"
#include "falconsCommon.hpp"
#include "FalconsRtDB2.hpp"

using std::exception;

int main(int argc, char **argv)
{
    try
    {
        int myRobotId = getRobotNumber();
        auto teamChar = getTeamChar();
        auto rtdb = RtDB2Store::getInstance().getRtDB2(myRobotId, teamChar);

        // Fetch configuration from yaml file
        std::string configFile = determineConfig("heartBeat");
        ConfigRTDBAdapter<T_CONFIG_HEARTBEATCOACH>* configAdapter = new ConfigRTDBAdapter<T_CONFIG_HEARTBEATCOACH>(CONFIG_HEARTBEATCOACH);
        configAdapter->loadYAML(configFile);

        T_CONFIG_HEARTBEATCOACH config;

        /*
         * Create loop for heart beat generation
         */
        while(true)
        {
            // Get configuration data inside loop to allow live update
            configAdapter->get(config);
            float sleepMs = 1000.0 / config.updateFrequency;

            std::chrono::system_clock::time_point end_time = std::chrono::system_clock::now() + std::chrono::milliseconds((int)sleepMs);

            //TRACE("heartbeat LATENCY start");
            T_HEARTBEAT_COACH beat = 0;
            rtdb->put(HEARTBEAT_COACH, &beat);

            std::this_thread::sleep_until(end_time);
        }

        delete configAdapter;
    }
    catch (exception &e)
    {
    	throw e;
    }

}
